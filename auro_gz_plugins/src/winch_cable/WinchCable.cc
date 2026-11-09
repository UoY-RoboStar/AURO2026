#include "WinchCable.hh"

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>

namespace
{
const char kBlockVisual[] = "crane::winch_block";
const char kWinchVisual[] = "crane::winch";
constexpr double kHalfHeight = 0.125;  // the block and the winch are both 0.25 m tall
constexpr double kThickness = 0.06;    // m
constexpr double kMinLength = 0.001;   // m; shorter than this, the cable is hidden
}  // namespace

namespace auro
{

void WinchCable::LoadConfig(const tinyxml2::XMLElement *)
{
  gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
}

bool WinchCable::eventFilter(QObject * _obj, QEvent * _event)
{
  if (_event->type() == gz::gui::events::Render::kType) {
    this->Update();
  }
  return QObject::eventFilter(_obj, _event);
}

void WinchCable::Update()
{
  if (!this->scene) {
    this->scene = gz::rendering::sceneFromFirstRenderEngine();
    if (!this->scene) {
      return;
    }
  }
  // The links appear in the scene some time after the GUI starts
  if (!this->block || !this->winch) {
    this->block = this->scene->VisualByName(kBlockVisual);
    this->winch = this->scene->VisualByName(kWinchVisual);
    if (!this->block || !this->winch) {
      return;
    }
  }
  if (!this->cable) {
    auto material = this->scene->CreateMaterial();
    material->SetAmbient(0.08, 0.08, 0.08);
    material->SetDiffuse(0.08, 0.08, 0.08);
    this->cable = this->scene->CreateVisual("winch_cable");
    this->cable->AddGeometry(this->scene->CreateBox());
    this->cable->SetMaterial(material);
    this->scene->RootVisual()->AddChild(this->cable);
    gzmsg << "Winch cable: drawing between " << kBlockVisual << " and " << kWinchVisual << "\n";
  }

  const auto top = this->block->WorldPosition() - gz::math::Vector3d(0, 0, kHalfHeight);
  const auto bottom = this->winch->WorldPosition() + gz::math::Vector3d(0, 0, kHalfHeight);
  const double length = top.Z() - bottom.Z();
  this->cable->SetVisible(length > kMinLength);
  if (length > kMinLength) {
    this->cable->SetLocalScale(kThickness, kThickness, length);
    this->cable->SetWorldPosition((top + bottom) / 2);
  }
}

}  // namespace auro

GZ_ADD_PLUGIN(auro::WinchCable, gz::gui::Plugin)
