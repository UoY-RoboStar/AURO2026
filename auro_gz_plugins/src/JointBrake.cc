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

/// \brief A model system that brakes one of its joints: while the brake is on, the joint's child
/// link is welded to its parent link where it is, so the joint does not move, whatever forces
/// act on it (gravity, or efforts commanded through ros2_control). Like a hoist's fail-safe
/// brake, it is on from the start unless `<engaged>` is false.
///
/// A weld, not a velocity command, because a joint's force command takes precedence over its
/// velocity command in gz-sim, and gz_ros2_control sets the force of an effort-controlled joint on
/// every step.
///
/// ## Services
///
/// Both use request gz.msgs.WorldControl (contents ignored) and reply gz.msgs.Boolean, the type
/// pair that ros_gz_bridge maps to ros_gz_interfaces/srv/ControlWorld, as gz_link_attacher does.
/// The reply's `data` is true once the brake is in the state asked for, including when it
/// already was. A request is carried out in the next PreUpdate, which gz-sim also runs while
/// paused; if that does not happen within 2 s, the call fails and the request is dropped.
///
/// - engage: put the brake on, holding the joint where it is now.
/// - release: take it off.
///
/// ## System parameters
///
/// - `<joint_name>` (required): the joint of this model to brake.
/// - `<engaged>` (optional, default true): whether the brake is on at the start, and after a
///   world reset.
/// - `<engage_service>` (optional): default `/model/<model>/joint/<joint>/brake/engage`.
/// - `<release_service>` (optional): default `/model/<model>/joint/<joint>/brake/release`.

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <chrono>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <string>

namespace auro
{
using namespace gz;
using namespace gz::sim;

class JointBrake
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
    const Model model(_entity);
    if (!model.Valid(_ecm))
    {
      gzerr << "[JointBrake] must be attached to a model. Not loading.\n";
      return;
    }
    const std::string modelName = model.Name(_ecm);

    this->jointName = _sdf->Get<std::string>("joint_name", "").first;
    const Entity joint = model.JointByName(_ecm, this->jointName);
    if (joint == kNullEntity)
    {
      gzerr << "[JointBrake] <joint_name> [" << this->jointName << "] not found in model ["
            << modelName << "]. Not loading.\n";
      return;
    }
    const auto *parentName = _ecm.Component<components::ParentLinkName>(joint);
    const auto *childName = _ecm.Component<components::ChildLinkName>(joint);
    this->parentLink = parentName ? model.LinkByName(_ecm, parentName->Data()) : kNullEntity;
    this->childLink = childName ? model.LinkByName(_ecm, childName->Data()) : kNullEntity;
    if (this->parentLink == kNullEntity || this->childLink == kNullEntity)
    {
      gzerr << "[JointBrake] joint [" << this->jointName << "] must join two links of model ["
            << modelName << "]. Not loading.\n";
      return;
    }

    this->engagedAtStart = _sdf->Get<bool>("engaged", true).first;
    this->wantEngaged = this->engagedAtStart;

    const std::string prefix = "/model/" + modelName + "/joint/" + this->jointName + "/brake/";
    const auto engageSrv = _sdf->Get<std::string>("engage_service", prefix + "engage").first;
    const auto releaseSrv = _sdf->Get<std::string>("release_service", prefix + "release").first;
    if (!this->node.Advertise(engageSrv, &JointBrake::OnEngage, this) ||
        !this->node.Advertise(releaseSrv, &JointBrake::OnRelease, this))
    {
      gzerr << "[JointBrake] failed to advertise [" << engageSrv << "] or [" << releaseSrv
            << "]. Not loading.\n";
      return;
    }
    gzmsg << "[JointBrake] " << modelName << "::" << this->jointName << ": services ["
          << engageSrv << "], [" << releaseSrv << "], "
          << (this->engagedAtStart ? "engaged" : "released") << " at start\n";
    this->valid = true;
  }

  void PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm) override
  {
    if (!this->valid)
      return;

    std::deque<std::shared_ptr<Request>> requests;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      requests.swap(this->pending);
    }
    for (auto &req : requests)
    {
      this->wantEngaged = req->engage;
      req->result.set_value(true);
    }

    // Also puts the brake on at the start, on the first step
    if (this->wantEngaged && this->weld == kNullEntity)
    {
      this->weld = _ecm.CreateEntity();
      _ecm.CreateComponent(
        this->weld, components::DetachableJoint({this->parentLink, this->childLink, "fixed"}));
      gzmsg << "[JointBrake] " << this->jointName << ": brake engaged.\n";
    }
    else if (!this->wantEngaged && this->weld != kNullEntity)
    {
      this->RemoveWeld(_ecm);
      gzmsg << "[JointBrake] " << this->jointName << ": brake released.\n";
    }
  }

  void Reset(const UpdateInfo &, EntityComponentManager &_ecm) override
  {
    // Back to as it starts: the next PreUpdate welds the joint where the reset left it
    if (this->weld != kNullEntity)
      this->RemoveWeld(_ecm);
    this->wantEngaged = this->engagedAtStart;
    std::lock_guard<std::mutex> lock(this->mutex);
    for (auto &req : this->pending)
      req->result.set_value(false);
    this->pending.clear();
  }

private:
  struct Request
  {
    bool engage;
    std::promise<bool> result;
  };

  bool OnEngage(const msgs::WorldControl &, msgs::Boolean &_rep)
  {
    _rep.set_data(this->Submit(true));
    return true;
  }

  bool OnRelease(const msgs::WorldControl &, msgs::Boolean &_rep)
  {
    _rep.set_data(this->Submit(false));
    return true;
  }

  /// Runs on a transport thread: queue the request for the next PreUpdate and wait for it.
  bool Submit(bool _engage)
  {
    auto req = std::make_shared<Request>();
    req->engage = _engage;
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
        gzwarn << "[JointBrake] " << (_engage ? "engage" : "release")
               << " timed out: no simulation update within 2 s.\n";
        return false;
      }
    }
    return result.get();  // PreUpdate took it just before we withdrew it
  }

  void RemoveWeld(EntityComponentManager &_ecm)
  {
    if (_ecm.HasEntity(this->weld))
      _ecm.RequestRemoveEntity(this->weld);
    this->weld = kNullEntity;
  }

  static constexpr std::chrono::seconds kRequestTimeout{2};

  std::string jointName;
  Entity parentLink{kNullEntity};
  Entity childLink{kNullEntity};
  Entity weld{kNullEntity};
  bool engagedAtStart{true};
  bool wantEngaged{true};
  bool valid{false};

  transport::Node node;
  std::mutex mutex;
  std::deque<std::shared_ptr<Request>> pending;
};
}  // namespace auro

GZ_ADD_PLUGIN(auro::JointBrake,
              gz::sim::System,
              auro::JointBrake::ISystemConfigure,
              auro::JointBrake::ISystemPreUpdate,
              auro::JointBrake::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(auro::JointBrake, "auro::JointBrake")
