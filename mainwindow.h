/**
 * @file mainwindow.h
 * @author Muhammad Ahmed Humais
 *         muhammad.humais@ku.ac.ae
 *         Khalifa University
 * @brief Main Window header file for user interface callbacks
 * @date 2020/09/16
 * @version v1.0
 * @package hospital_robot
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/tool_manager.h"
#include "rviz/tool.h"

#include "rosthread.h"
#include "emgthread.h"
#include "statusthread.h"
#include "goalthread.h"
#include "tempthread.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

namespace rviz
{
class Display;
//class PathDisplay;
class RenderPanel;
class VisualizationManager;
class ToolManager;
class Tool;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    rosThread rthread;
    emgThread ethread;
    statusThread sthread;
    goalThread gthread;
    tempThread tthread;

private slots:
    void update_robot_status(QString con, QString bat, QString pow_draw);
    void update_auto_status(QString stat);

    void on_speedSlider_valueChanged(int value);

    void on_fwdButton_pressed();

    void on_fwdButton_released();

    void on_bwdButton_pressed();

    void on_bwdButton_released();

    void on_rightButton_pressed();

    void on_rightButton_released();

    void on_leftButton_pressed();

    void on_leftButton_released();

    void on_stopButton_pressed();

    void on_initialButton_released();

    void on_goalButton_released();

    void on_cancelButton_pressed();

    void on_emgButton_toggled(bool checked);

    void on_autoCheck_toggled(bool checked);

    void on_tempButton_toggled(bool checked);

private:
    Ui::MainWindow *ui;
    bool eventFilter(QObject *o, QEvent *e);
    void enable_auto(bool enable);

    rviz::VisualizationManager *manager_;
    rviz::RenderPanel *render_panel_;
    rviz::Display *map_, *robotModel_, *goal_position_, *plan_, *cam_;
    rviz::ToolManager *tmanager_;
    rviz::Tool *goalTool_;
    rviz::Tool *initialTool_;
};

#endif // MAINWINDOW_H
