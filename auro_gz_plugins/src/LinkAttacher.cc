// Copyright 2026 Pedro Ribeiro
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \brief A model system that, on request, welds one of its links (the "parent link") with a
/// fixed joint to the nearest link of another, non-static model, and removes that joint again.
///
/// Unlike gz::sim::systems::DetachableJoint, nothing is attached at startup and the child is
/// chosen at runtime, so a single instance serves any number of objects.
///
/// ## Services
///
/// Both use request gz.msgs.WorldControl (contents ignored) and reply gz.msgs.Boolean, the
/// type pair that ros_gz_bridge maps to ros_gz_interfaces/srv/ControlWorld. The reply's `data`
/// is true on success; the reason for a failure is written to the Gazebo log. A request is
/// carried out in the next PreUpdate, which gz-sim also runs while paused; if that does not
/// happen within 2 s (e.g. the server is stalled) the call fails and the request is dropped.
///
/// - attach: attach the nearest attachable link whose origin is within `<max_distance>` of the
///   parent link's origin. Fails if something is already attached or nothing is in reach.
/// - detach: remove the joint. Fails if nothing is attached.
///
/// ## System parameters
///
/// - `<parent_link>` (required): link of this model that objects are attached to.
/// - `<max_distance>` (required, metres, > 0): reach, measured between link origins.
/// - `<attach_service>` (optional): default `/model/<model>/link_attacher/attach`.
/// - `<detach_service>` (optional): default `/model/<model>/link_attacher/detach`.
/// - `<publish_state>` (optional, default false): if true, publish which link is held on
///   `<state_topic>` (gz.msgs.StringMsg): `"detached"` or `"attached:<model>::<link>"`, on every
///   change and once per second of sim time. If false, the topic is not advertised at all.
/// - `<state_topic>` (optional): default `/model/<model>/link_attacher/state`.
/// - `<failure_probability>` (optional, 0-1, default 0): chance that an attach or detach that
///   would otherwise succeed fails: nothing changes and the service replies false, so the caller
///   has to check the reply and retry. A warning goes to the Gazebo log.
/// - `<random_seed>` (optional, integer >= 0): seed for those failures, which then repeat for the
///   same sequence of requests. Without it, they differ on every run.
///
/// Warnings (shown at the default -v2 log level) never name the held link; the info messages
/// on attach/detach (-v3 and above) do.
///
/// A link is attachable if it is not part of this model and no model containing it is static.
/// The joint keeps the links at their current relative pose; nothing is moved.

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Static.hh>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <chrono>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <random>
#include <string>

namespace auro
{
using namespace gz;
using namespace gz::sim;

class LinkAttacher
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemReset
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    this->model = Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      gzerr << "[LinkAttacher] must be attached to a model. Not loading.\n";
      return;
    }
    const std::string modelName = this->model.Name(_ecm);

    const auto parentLinkName = _sdf->Get<std::string>("parent_link", "").first;
    this->parentLink = this->model.LinkByName(_ecm, parentLinkName);
    if (this->parentLink == kNullEntity)
    {
      gzerr << "[LinkAttacher] <parent_link> [" << parentLinkName << "] not found in model ["
            << modelName << "]. Not loading.\n";
      return;
    }

    this->maxDistance = _sdf->Get<double>("max_distance", 0.0).first;
    if (this->maxDistance <= 0.0)
    {
      gzerr << "[LinkAttacher] <max_distance> must be > 0. Not loading.\n";
      return;
    }

    this->failureProbability = _sdf->Get<double>("failure_probability", 0.0).first;
    if (this->failureProbability < 0.0 || this->failureProbability > 1.0)
    {
      gzerr << "[LinkAttacher] <failure_probability> must be between 0 and 1. Not loading.\n";
      return;
    }
    if (_sdf->HasElement("random_seed"))
      this->rng.seed(_sdf->Get<int>("random_seed"));
    else
      this->rng.seed(std::random_device{}());

    const std::string prefix = "/model/" + modelName + "/link_attacher/";
    const auto attachSrv = _sdf->Get<std::string>("attach_service", prefix + "attach").first;
    const auto detachSrv = _sdf->Get<std::string>("detach_service", prefix + "detach").first;
    const auto stateTopic = _sdf->Get<std::string>("state_topic", prefix + "state").first;
    this->publishState = _sdf->Get<bool>("publish_state", false).first;

    if (!this->node.Advertise(attachSrv, &LinkAttacher::OnAttach, this) ||
        !this->node.Advertise(detachSrv, &LinkAttacher::OnDetach, this))
    {
      gzerr << "[LinkAttacher] failed to advertise [" << attachSrv << "] or [" << detachSrv
            << "]. Not loading.\n";
      return;
    }
    if (this->publishState)
      this->statePub = this->node.Advertise<msgs::StringMsg>(stateTopic);

    gzmsg << "[LinkAttacher] " << modelName << "::" << parentLinkName << ": services ["
          << attachSrv << "], [" << detachSrv << "], state "
          << (this->publishState ? "[" + stateTopic + "]" : std::string("not published"))
          << ", max_distance " << this->maxDistance << " m, failure probability "
          << this->failureProbability << "\n";
    this->valid = true;
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (!this->valid)
      return;
    this->simTime = _info.simTime;

    // The child may have been removed from the world while attached.
    if (this->joint != kNullEntity && !_ecm.HasEntity(this->childLink))
    {
      gzwarn << "[LinkAttacher] the attached link disappeared; detaching.\n";
      this->RemoveJoint(_ecm);
    }

    std::deque<std::shared_ptr<Request>> requests;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      requests.swap(this->pending);
    }
    for (auto &req : requests)
      req->result.set_value(req->attach ? this->Attach(_ecm) : this->Detach(_ecm));

    if (this->publishState && this->simTime - this->lastStatePub >= std::chrono::seconds(1))
      this->PublishState();
  }

  void Reset(const UpdateInfo &, EntityComponentManager &_ecm) override
  {
    if (this->joint != kNullEntity)
      this->RemoveJoint(_ecm);
    std::lock_guard<std::mutex> lock(this->mutex);
    for (auto &req : this->pending)
      req->result.set_value(false);
    this->pending.clear();
  }

private:
  struct Request
  {
    bool attach;
    std::promise<bool> result;
  };

  bool OnAttach(const msgs::WorldControl &, msgs::Boolean &_rep)
  {
    _rep.set_data(this->Submit(true));
    return true;
  }

  bool OnDetach(const msgs::WorldControl &, msgs::Boolean &_rep)
  {
    _rep.set_data(this->Submit(false));
    return true;
  }

  /// Runs on a transport thread: queue the request for the next PreUpdate and wait for it.
  bool Submit(bool _attach)
  {
    auto req = std::make_shared<Request>();
    req->attach = _attach;
    auto result = req->result.get_future();
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->pending.push_back(req);
    }
    if (result.wait_for(kRequestTimeout) == std::future_status::ready)
      return result.get();

    // Timed out: withdraw the request so it is not carried out after we have replied.
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      auto it = std::find(this->pending.begin(), this->pending.end(), req);
      if (it != this->pending.end())
      {
        this->pending.erase(it);
        gzwarn << "[LinkAttacher] " << (_attach ? "attach" : "detach")
               << " timed out: no simulation update within 2 s.\n";
        return false;
      }
    }
    return result.get();  // PreUpdate took it just before we withdrew it
  }

  bool Attach(EntityComponentManager &_ecm)
  {
    if (this->joint != kNullEntity)
    {
      gzwarn << "[LinkAttacher] attach refused: already holding something; detach first.\n";
      return false;
    }

    double dist{0.0};
    const Entity child = this->NearestLink(_ecm, dist);
    if (child == kNullEntity)
    {
      gzwarn << "[LinkAttacher] attach refused: nothing attachable within " << this->maxDistance
             << " m of the winch.\n";
      return false;
    }
    if (this->MechanismFails("attach"))
      return false;

    this->joint = _ecm.CreateEntity();
    _ecm.CreateComponent(this->joint,
                         components::DetachableJoint({this->parentLink, child, "fixed"}));
    this->childLink = child;
    // scopedName starts with the world name; drop it.
    this->childName = scopedName(child, _ecm, "::", false);
    if (const auto sep = this->childName.find("::"); sep != std::string::npos)
      this->childName.erase(0, sep + 2);
    gzmsg << "[LinkAttacher] attached [" << this->childName << "] at " << dist << " m.\n";
    this->PublishState();
    return true;
  }

  bool Detach(EntityComponentManager &_ecm)
  {
    if (this->joint == kNullEntity)
    {
      gzwarn << "[LinkAttacher] detach refused: nothing is attached.\n";
      return false;
    }
    if (this->MechanismFails("detach"))
      return false;
    this->RemoveJoint(_ecm);
    return true;
  }

  /// Decide whether an otherwise valid attach or detach fails.
  bool MechanismFails(const char *_action)
  {
    if (std::uniform_real_distribution<double>(0.0, 1.0)(this->rng) >= this->failureProbability)
      return false;
    gzwarn << "[LinkAttacher] " << _action << " failed: mechanism failure.\n";
    return true;
  }

  void RemoveJoint(EntityComponentManager &_ecm)
  {
    if (_ecm.HasEntity(this->joint))
      _ecm.RequestRemoveEntity(this->joint);
    gzmsg << "[LinkAttacher] detached [" << this->childName << "].\n";
    this->joint = kNullEntity;
    this->childLink = kNullEntity;
    this->childName.clear();
    this->PublishState();
  }

  /// Not part of this model, and no model containing it is static (a fixed joint to a static
  /// body would pin this model in place).
  bool Attachable(Entity _link, const EntityComponentManager &_ecm) const
  {
    if (topLevelModel(_link, _ecm) == this->model.Entity())
      return false;
    for (Entity e = _link; e != kNullEntity;)
    {
      if (_ecm.Component<components::Model>(e))
      {
        const auto *isStatic = _ecm.Component<components::Static>(e);
        if (isStatic && isStatic->Data())
          return false;
      }
      const auto *parent = _ecm.Component<components::ParentEntity>(e);
      e = parent ? parent->Data() : kNullEntity;
    }
    return true;
  }

  Entity NearestLink(const EntityComponentManager &_ecm, double &_dist) const
  {
    const auto from = worldPose(this->parentLink, _ecm).Pos();
    Entity best{kNullEntity};
    _dist = this->maxDistance;
    _ecm.Each<components::Link>(
      [&](const Entity &_e, const components::Link *) -> bool
      {
        const double d = (worldPose(_e, _ecm).Pos() - from).Length();
        if (d <= _dist && this->Attachable(_e, _ecm))
        {
          best = _e;
          _dist = d;
        }
        return true;
      });
    return best;
  }

  void PublishState()
  {
    if (!this->publishState)
      return;
    msgs::StringMsg msg;
    msg.set_data(this->joint == kNullEntity ? "detached" : "attached:" + this->childName);
    this->statePub.Publish(msg);
    this->lastStatePub = this->simTime;  // drives the 1 Hz heartbeat
  }

  static constexpr std::chrono::seconds kRequestTimeout{2};

  Model model{kNullEntity};
  Entity parentLink{kNullEntity};
  Entity joint{kNullEntity};
  Entity childLink{kNullEntity};
  std::string childName;
  double maxDistance{0.0};
  double failureProbability{0.0};
  std::mt19937 rng;
  bool valid{false};
  bool publishState{false};

  transport::Node node;
  transport::Node::Publisher statePub;
  std::chrono::steady_clock::duration simTime{0};
  std::chrono::steady_clock::duration lastStatePub{0};

  std::mutex mutex;
  std::deque<std::shared_ptr<Request>> pending;
};
}  // namespace auro

GZ_ADD_PLUGIN(auro::LinkAttacher,
              gz::sim::System,
              auro::LinkAttacher::ISystemConfigure,
              auro::LinkAttacher::ISystemPreUpdate,
              auro::LinkAttacher::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(auro::LinkAttacher, "auro::LinkAttacher")
