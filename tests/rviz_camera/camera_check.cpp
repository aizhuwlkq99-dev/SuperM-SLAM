#include <QApplication>
#include <QElapsedTimer>
#include <QMouseEvent>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <iostream>
#include <stdexcept>

void pump(int ms) {
  QElapsedTimer timer;
  timer.start();
  while (timer.elapsed() < ms) { QApplication::processEvents(); QThread::msleep(5); }
}
void require(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}
void drag(rviz_common::RenderPanel* panel, Qt::MouseButton button, QPoint delta) {
  const QPoint start = panel->rect().center();
  QMouseEvent press(QEvent::MouseButtonPress, start, button, button, Qt::NoModifier);
  QApplication::sendEvent(panel, &press);
  QMouseEvent move(QEvent::MouseMove, start + delta, Qt::NoButton, button, Qt::NoModifier);
  QApplication::sendEvent(panel, &move);
  QMouseEvent release(QEvent::MouseButtonRelease, start + delta, button, Qt::NoButton, Qt::NoModifier);
  QApplication::sendEvent(panel, &release);
  pump(150);
}
int main(int argc, char** argv) {
  if (argc < 2) return 2;
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  try {
    auto node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("superm_camera_check");
    rviz_common::VisualizationFrame frame(node);
    frame.setApp(&app);
    frame.setSplashPath("");
    frame.initialize(node, QString::fromLocal8Bit(argv[1]));
    frame.setWindowTitle("SuperM-LIO camera interaction check");
    frame.show();
    pump(1000);
    auto* manager = frame.getManager();
    auto* tool = manager->getToolManager()->getCurrentTool();
    require(tool && tool->getClassId() == "rviz_default_plugins/MoveCamera", "MoveCamera is not the active tool");
    auto* panel = frame.findChild<rviz_common::RenderPanel*>();
    require(panel, "No RViz render panel");
    auto* view = manager->getViewManager()->getCurrent();
    const double yaw_before = view->subProp("Yaw")->getValue().toDouble();
    drag(panel, Qt::LeftButton, QPoint(80, 30));
    const double yaw_after = view->subProp("Yaw")->getValue().toDouble();
    require(std::abs(yaw_after - yaw_before) > 0.01, "Left drag did not rotate camera");
    auto* focal = view->subProp("Focal Point");
    const double x_before = focal->subProp("X")->getValue().toDouble();
    const double y_before = focal->subProp("Y")->getValue().toDouble();
    drag(panel, Qt::MiddleButton, QPoint(60, 40));
    const double x_after = focal->subProp("X")->getValue().toDouble();
    const double y_after = focal->subProp("Y")->getValue().toDouble();
    require(std::hypot(x_after - x_before, y_after - y_before) > 0.01, "Middle drag did not pan camera");
    const double distance_before = view->subProp("Distance")->getValue().toDouble();
    drag(panel, Qt::RightButton, QPoint(0, 60));
    const double distance_after = view->subProp("Distance")->getValue().toDouble();
    require(std::abs(distance_after - distance_before) > 0.01, "Right drag did not zoom camera");
    if (argc > 2) frame.grab().save(argv[2]);
    std::cout << "PASS: MoveCamera active; left drag rotates, middle drag pans, right drag zooms" << std::endl;
    // Closing a modified RViz frame prompts to save; destroying our test frame avoids that dialog.
    frame.hide();
  } catch(const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
