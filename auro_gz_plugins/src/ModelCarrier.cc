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

/// \brief A model system that, on request, carries another top-level model at a fixed pose
/// relative to its own, and puts it down again at a given pose in the world.
///
/// The carried model is moved kinematically: on every simulation step in which this model has
/// moved, the carried model is sent to its pose relative to this one, the same way Gazebo's
/// set_pose service moves models. Nothing joins the two, so the carried model adds no mass or
/// inertia to this one and their collisions never interact through a joint. It is meant for
/// static models, whose collisions Gazebo moves along with them; a dynamic one would still be
/// subject to gravity and contacts between steps.
///
/// ## Services
///
/// carry and release use request gz.msgs.Pose and reply gz.msgs.Boolean. The reply's `data` is true on
/// success; the reason for a failure is written to the Gazebo log. A request is carried out in
/// the next PreUpdate, which gz-sim also runs while paused; if that does not happen within 2 s
/// (e.g. the server is stalled) the call fails and the request is dropped.
///
/// - carry: `name` is the top-level model to carry, `position` and `orientation` its pose in
///   this model's frame. It is moved there at once. Fails if something is already carried, or
///   there is no such model, or it is this model, or it is out of reach.
/// - release: stop carrying, and move the carried model to `position` and `orientation` in the
///   world (`name` is ignored). Fails if nothing is carried, or the pose is out of reach.
/// - pose: request gz.msgs.Empty, reply gz.msgs.Pose: this model's pose in the world, named
///   after it, with the simulation time it is from in `header.stamp`. Answered at once, from the
///   last simulation step; fails before the first one. It lets a client follow this model by
///   asking, rather than by subscribing to the world's pose topics: in Python, a gz-transport
///   request does not get its reply while the same process has a subscription.
///
/// ## System parameters
///
/// - `<carry_service>` (optional): default `/model/<model>/model_carrier/carry`.
/// - `<release_service>` (optional): default `/model/<model>/model_carrier/release`.
/// - `<pose_service>` (optional): default `/model/<model>/model_carrier/pose`.
/// - `<max_distance>` (optional, metres, > 0): reach, measured horizontally between this model's
///   origin and the origin of the model to carry, or the pose to release it at. Without it, the
///   reach is unlimited. The services can be called by anything that talks to Gazebo, so this
///   keeps them from moving models from, or to, anywhere in the world.

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace auro
{
using namespace gz;
using namespace gz::sim;

class ModelCarrier
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
      gzerr << "[ModelCarrier] must be attached to a model. Not loading.\n";
      return;
    }
    const std::string prefix = "/model/" + this->model.Name(_ecm) + "/model_carrier/";
    const auto carrySrv = _sdf->Get<std::string>("carry_service", prefix + "carry").first;
    const auto releaseSrv = _sdf->Get<std::string>("release_service", prefix + "release").first;
    const auto poseSrv = _sdf->Get<std::string>("pose_service", prefix + "pose").first;
    this->modelName = this->model.Name(_ecm);
    if (_sdf->HasElement("max_distance"))
    {
      this->maxDistance = _sdf->Get<double>("max_distance");
      if (this->maxDistance <= 0.0)
      {
        gzerr << "[ModelCarrier] <max_distance> must be > 0. Not loading.\n";
        return;
      }
    }
    if (!this->node.Advertise(carrySrv, &ModelCarrier::OnCarry, this) ||
        !this->node.Advertise(releaseSrv, &ModelCarrier::OnRelease, this) ||
        !this->node.Advertise(poseSrv, &ModelCarrier::OnPose, this))
    {
      gzerr << "[ModelCarrier] failed to advertise [" << carrySrv << "], [" << releaseSrv
            << "] or [" << poseSrv << "]. Not loading.\n";
      return;
    }
    gzmsg << "[ModelCarrier] " << this->model.Name(_ecm) << ": services [" << carrySrv
          << "], [" << releaseSrv << "], [" << poseSrv << "], max_distance "
          << (this->maxDistance > 0.0 ? std::to_string(this->maxDistance) + " m" : "unlimited")
          << "\n";
    this->valid = true;
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (!this->valid)
      return;
    {
      std::lock_guard<std::mutex> lock(this->poseMutex);
      this->lastPose = worldPose(this->model.Entity(), _ecm);
      this->lastPoseTime = _info.simTime;
    }

    // The carried model may have been removed from the world.
    if (this->carried != kNullEntity && !_ecm.HasEntity(this->carried))
    {
      gzwarn << "[ModelCarrier] the carried model disappeared.\n";
      this->carried = kNullEntity;
    }

    std::deque<std::shared_ptr<Request>> requests;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      requests.swap(this->pending);
    }
    for (auto &req : requests)
      req->result.set_value(req->carry ? this->Carry(*req, _ecm) : this->Release(*req, _ecm));

    if (this->carried == kNullEntity)
      return;
    // This model's pose as of the last physics step; the carried model follows it into the next
    const auto pose = worldPose(this->model.Entity(), _ecm);
    if (this->lastCarrierPose && pose.Equal(*this->lastCarrierPose, kTolerance))
      return;
    Model(this->carried).SetWorldPoseCmd(_ecm, pose * this->offset);
    this->lastCarrierPose = pose;
  }

  void Reset(const UpdateInfo &, EntityComponentManager &) override
  {
    this->carried = kNullEntity;
    std::lock_guard<std::mutex> lock(this->mutex);
    for (auto &req : this->pending)
      req->result.set_value(false);
    this->pending.clear();
  }

private:
  struct Request
  {
    bool carry;
    std::string name;
    math::Pose3d pose;
    std::promise<bool> result;
  };

  bool OnCarry(const msgs::Pose &_req, msgs::Boolean &_rep)
  {
    _rep.set_data(this->Submit(true, _req));
    return true;
  }

  bool OnRelease(const msgs::Pose &_req, msgs::Boolean &_rep)
  {
    _rep.set_data(this->Submit(false, _req));
    return true;
  }

  bool OnPose(const msgs::Empty &, msgs::Pose &_rep)
  {
    std::lock_guard<std::mutex> lock(this->poseMutex);
    if (!this->lastPose)
      return false;
    _rep = msgs::Convert(*this->lastPose);
    _rep.set_name(this->modelName);
    *_rep.mutable_header()->mutable_stamp() = msgs::Convert(this->lastPoseTime);
    return true;
  }

  /// Runs on a transport thread: queue the request for the next PreUpdate and wait for it.
  bool Submit(bool _carry, const msgs::Pose &_msg)
  {
    auto req = std::make_shared<Request>();
    req->carry = _carry;
    req->name = _msg.name();
    req->pose = msgs::Convert(_msg);
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
        gzwarn << "[ModelCarrier] " << (_carry ? "carry" : "release")
               << " timed out: no simulation update within 2 s.\n";
        return false;
      }
    }
    return result.get();  // PreUpdate took it just before we withdrew it
  }

  bool Carry(const Request &_req, EntityComponentManager &_ecm)
  {
    if (this->carried != kNullEntity)
    {
      gzwarn << "[ModelCarrier] carry refused: already carrying a model; release it first.\n";
      return false;
    }
    const Entity entity = _ecm.EntityByComponents(
      components::Name(_req.name), components::Model(),
      components::ParentEntity(worldEntity(_ecm)));
    if (entity == kNullEntity || entity == this->model.Entity())
    {
      gzwarn << "[ModelCarrier] carry refused: no model [" << _req.name << "] to carry.\n";
      return false;
    }
    if (!this->InReach(worldPose(entity, _ecm).Pos(), _ecm))
    {
      gzwarn << "[ModelCarrier] carry refused: [" << _req.name << "] is out of reach.\n";
      return false;
    }
    this->carried = entity;
    this->offset = _req.pose;
    this->lastCarrierPose.reset();  // moves it in this same PreUpdate
    gzmsg << "[ModelCarrier] carrying [" << _req.name << "].\n";
    return true;
  }

  bool Release(const Request &_req, EntityComponentManager &_ecm)
  {
    if (this->carried == kNullEntity)
    {
      gzwarn << "[ModelCarrier] release refused: nothing is carried.\n";
      return false;
    }
    if (!this->InReach(_req.pose.Pos(), _ecm))
    {
      gzwarn << "[ModelCarrier] release refused: the pose is out of reach.\n";
      return false;
    }
    Model(this->carried).SetWorldPoseCmd(_ecm, _req.pose);
    gzmsg << "[ModelCarrier] released the carried model.\n";
    this->carried = kNullEntity;
    return true;
  }

  /// Whether a point in the world is within <max_distance> of this model's origin, horizontally.
  bool InReach(const math::Vector3d &_point, const EntityComponentManager &_ecm) const
  {
    if (this->maxDistance <= 0.0)
      return true;
    const auto origin = worldPose(this->model.Entity(), _ecm).Pos();
    return std::hypot(_point.X() - origin.X(), _point.Y() - origin.Y()) <= this->maxDistance;
  }

  static constexpr std::chrono::seconds kRequestTimeout{2};
  static constexpr double kTolerance{1e-6};

  Model model{kNullEntity};
  Entity carried{kNullEntity};
  math::Pose3d offset;
  double maxDistance{0.0};  // 0: unlimited
  std::optional<math::Pose3d> lastCarrierPose;
  bool valid{false};

  std::string modelName;
  std::mutex poseMutex;  // guards the two below, read by the pose service's transport thread
  std::optional<math::Pose3d> lastPose;
  std::chrono::steady_clock::duration lastPoseTime{0};

  transport::Node node;
  std::mutex mutex;
  std::deque<std::shared_ptr<Request>> pending;
};
}  // namespace auro

GZ_ADD_PLUGIN(auro::ModelCarrier,
              gz::sim::System,
              auro::ModelCarrier::ISystemConfigure,
              auro::ModelCarrier::ISystemPreUpdate,
              auro::ModelCarrier::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(auro::ModelCarrier, "auro::ModelCarrier")
