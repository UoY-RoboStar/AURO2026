#ifndef GZ_WINCH_CABLE_WINCHCABLE_HH_
#define GZ_WINCH_CABLE_WINCHCABLE_HH_

#include <gz/gui/Plugin.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

namespace auro
{
/// \brief Draws the cable between the crane's winch block and winch.
///
/// A visual in the world cannot change length while the simulation runs, so this plugin adds its
/// own box to the GUI's scene and resizes it on every rendered frame, from the positions of the
/// two links in that scene. Camera sensors render their own scene, so they do not see it.
class WinchCable : public gz::gui::Plugin
{
  Q_OBJECT

public:
  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

protected:
  bool eventFilter(QObject * _obj, QEvent * _event) override;

private:
  /// \brief Find the scene and links if needed, then fit the cable between the links.
  void Update();

  gz::rendering::ScenePtr scene;
  gz::rendering::VisualPtr block;
  gz::rendering::VisualPtr winch;
  gz::rendering::VisualPtr cable;
};
}  // namespace auro

#endif  // GZ_WINCH_CABLE_WINCHCABLE_HH_
