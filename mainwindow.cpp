#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    qDebug()<< "UI ready";

    // setting up rendering area
    render_panel_ = new rviz::RenderPanel();
    ui->renderLayout->addWidget(render_panel_);
    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );

    // setting view manager with fixed frame as "map"

    cam_ = manager_->createDisplay( "rviz/Image", "camera_feed", true );
    cam_->setTopic("camera/color/image_raw","sensor_msgs/Image");

    tmanager_ = manager_->getToolManager();
    manager_->initialize();
    manager_->setFixedFrame("map");
     //show map
    map_ = manager_->createDisplay( "rviz/Map", "Map", true );
    map_->subProp( "Topic")->setValue("map");

    // show robot model
    robotModel_ = manager_->createDisplay( "rviz/RobotModel", "Robot", true );

    // show goal position
    goal_position_ = manager_->createDisplay( "rviz/Pose", "Goal_position", true );
    goal_position_->subProp("Topic")->setValue("move_base_simple/goal");
    goal_position_->subProp("Shaft Length")->setValue(0.3);
    goal_position_->subProp("Shaft Radius")->setValue(0.1);
    goal_position_->subProp("Head Length")->setValue(0.15);
    goal_position_->subProp("Head Radius")->setValue(0.15);

    //show planned path
    plan_ = manager_->createDisplay( "rviz/Path", "Planned_path", true );
  //  plan_->setTopic("/move_base/NavfnROS/plan", "nav_msgs/Path");

    // show camera feed
//    cam_ = manager_->createDisplay( "rviz/Camera", "camera_feed", true );
//    cam_->subProp("Topic")->setValue("camera/color/image_raw");

    // setting up the tools

    initialTool_ = tmanager_->addTool("rviz/SetInitialPose");
    initialTool_->getPropertyContainer()->subProp( "Topic" )->setValue("/initialpose");
    goalTool_ = tmanager_->addTool("rviz/SetGoal");
    goalTool_->getPropertyContainer()->subProp( "Topic" )->setValue("move_base_simple/goal");

    manager_->startUpdate();
    qDebug()<< "Starting ROS";

    this->rthread.start();
    connect(&sthread, &statusThread::update_status, this, &MainWindow::update_robot_status);
    this->sthread.start();
    connect(&gthread, &goalThread::auto_status, this, &MainWindow::update_auto_status);
    this->gthread.start();

    QList<QWidget*> widgets = findChildren<QWidget*>();
    foreach (QWidget* widget, widgets)
      widget->installEventFilter(this);
}

MainWindow::~MainWindow()
{
    rthread.requestInterruption();
    sthread.requestInterruption();
    gthread.requestInterruption();
    manager_->removeAllDisplays();
    delete manager_;
    delete ui;
    rthread.wait();
    sthread.wait();
    gthread.wait();
}

bool MainWindow::eventFilter(QObject* o, QEvent* e)
{
    if (e->type() == QEvent::KeyPress)
    {
        QKeyEvent* k = static_cast<QKeyEvent*>(e);
        switch (k->key()) {
        case Qt::Key_Up:
            rthread.fwd.store(1);
            return true;
        case Qt::Key_Down:
            rthread.bwd.store(1);
            return true;
        case Qt::Key_Left:
            rthread.lft.store(1);
            return true;
        case Qt::Key_Right:
            rthread.rgt.store(1);
            return true;
        case Qt::Key_Space:
            rthread.stp.store(1);
            return true;
        }
    }
    else if (e->type() == QEvent::KeyRelease)
    {
        QKeyEvent* k = static_cast<QKeyEvent*>(e);
        switch (k->key()) {
        case Qt::Key_Up:
            rthread.fwd.store(0);
            return true;
        case Qt::Key_Down:
            rthread.bwd.store(0);
            return true;
        case Qt::Key_Left:
            rthread.lft.store(0);
            return true;
        case Qt::Key_Right:
            rthread.rgt.store(0);
            return true;
        }
    }
    return false;
}

void MainWindow::update_robot_status(QString con, QString bat, QString pow_draw)
{
    ui->conn_stat->setText(con);
    ui->conn_stat->setAlignment(Qt::AlignRight);
    ui->batt_stat->setText(bat);
    ui->batt_stat->setAlignment(Qt::AlignRight);
    ui->pow_stat->setText(pow_draw);
    ui->pow_stat->setAlignment(Qt::AlignRight);
}

void MainWindow::update_auto_status(QString stat)
{
    ui->auto_stat->setText(stat);
    ui->auto_stat->setAlignment(Qt::AlignRight);
}

void MainWindow::on_speedSlider_valueChanged(int value)
{
    rthread.max_trans_speed.store(value);
}

void MainWindow::on_fwdButton_pressed()
{
    rthread.fwd.store(1);
}

void MainWindow::on_fwdButton_released()
{
    rthread.fwd.store(0);
}

void MainWindow::on_bwdButton_pressed()
{
    rthread.bwd.store(1);
}

void MainWindow::on_bwdButton_released()
{
    rthread.bwd.store(0);
}

void MainWindow::on_rightButton_pressed()
{
    rthread.rgt.store(1);
}

void MainWindow::on_rightButton_released()
{
    rthread.rgt.store(0);
}

void MainWindow::on_leftButton_pressed()
{
    rthread.lft.store(1);
}

void MainWindow::on_leftButton_released()
{
    rthread.lft.store(0);
}

void MainWindow::on_stopButton_pressed()
{
    rthread.stp.store(1);
}

void MainWindow::on_initialButton_released()
{
    if(initialTool_ != NULL)
    {
        tmanager_->setCurrentTool(initialTool_);
    }
}

void MainWindow::on_goalButton_released()
{
    if( goalTool_ != NULL )
    {
    tmanager_->setCurrentTool(goalTool_);
    }
}

void MainWindow::on_cancelButton_pressed()
{
    gthread.cancel_.store(1);
    //cancel_goal action api to be called
}


void MainWindow::on_tempButton_released()
{
    //tempthread to be called
}

void MainWindow::enable_auto(bool enable){
    rthread.enbl.store(!enable);
    ui->goalButton->setEnabled(enable);
    ui->cancelButton->setEnabled(enable);
    ui->fwdButton->setEnabled(!enable);
    ui->bwdButton->setEnabled(!enable);
    ui->leftButton->setEnabled(!enable);
    ui->rightButton->setEnabled(!enable);
    ui->stopButton->setEnabled(!enable);
    gthread.enable_.store(enable);
    if(!enable){
        gthread.cancel_.store(1);
        ui->auto_stat->setText("manual");
        ui->auto_stat->setAlignment(Qt::AlignRight);
    }
}


void MainWindow::on_emgButton_toggled(bool checked)
{
    ethread.set_emg_state(checked);
    ethread.start(QThread::HighestPriority);
}

void MainWindow::on_autoCheck_toggled(bool checked)
{
    enable_auto(checked);
}
