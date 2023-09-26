#include "multibot_server/server_panel.hpp"

#include <iostream>
#include <cmath>

#include <arpa/inet.h>
#include <cerrno>
#include <ifaddrs.h>
#include <net/if.h>
#include <sysexits.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <QString>
#include <QVBoxLayout>
#include <QTabBar>
#include <QKeyEvent>

#include "multibot_server/ui_server_panel.h"

#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

void Server::Panel::attach(Observer::ObserverInterface<PanelUtil::Msg> &_observer)
{
    std::scoped_lock<std::mutex> lock(mtx_);

    if (std::find(list_observer_.begin(), list_observer_.end(), &_observer) == list_observer_.end())
        list_observer_.push_back(&_observer);
}

void Server::Panel::detach(Observer::ObserverInterface<PanelUtil::Msg> &_observer)
{
    std::scoped_lock<std::mutex> lock(mtx_);

    auto observer_iter = std::find(list_observer_.begin(), list_observer_.end(), &_observer);
    while (observer_iter != list_observer_.end())
    {
        list_observer_.remove(&_observer);
        observer_iter = std::find(list_observer_.begin(), list_observer_.end(), &_observer);
    }
}

void Server::Panel::notify()
{
    for (auto &observer : list_observer_)
        observer->update(msg_);
}

void Server::Panel::handleButton()
{
    QObject *button = QObject::sender();

    inactivatedRobot_ = activatedRobot_;
    activatedRobot_ = button->objectName().toStdString();

    const auto &robotDB = instance_manager_->getRobot(activatedRobot_);
    activatedRobotGoal_ = robotDB.robotInfo_.goal_.component_;
    activatedRobotModeState_ = robotDB.mode_;

    ui_->ServerTab->setTabEnabled(PanelUtil::Tab::ROBOT, true);
    ui_->ServerTab->currentChanged(PanelUtil::Tab::ROBOT);

    ui_->doubleSpinBox_goalX->setValue(activatedRobotGoal_.x);
    ui_->doubleSpinBox_goalY->setValue(activatedRobotGoal_.y);
    ui_->doubleSpinBox_goalYaw->setValue(activatedRobotGoal_.theta);

    switch (activatedRobotModeState_)
    {
    case PanelUtil::Mode::MANUAL:
    {
        ui_->pushButton_Mode->setText(QString::fromStdString("Manual"));
        ui_->pushButton_Mode->setStyleSheet(
            "color: rgb(255,190,11);\nborder: 2px solid rgb(255,190,11);\nborder-radius: 15px");
        break;
    }

    case PanelUtil::Mode::REMOTE:
    {
        ui_->pushButton_Mode->setText(QString::fromStdString("Remote"));
        ui_->pushButton_Mode->setStyleSheet(
            "color: rgb(0,213,255);\nborder: 2px solid rgb(0,213,255);\nborder-radius: 15px");

        activatedRobotLinVel_ = 0.0;
        activatedRobotAngVel_ = 0.0;

        break;
    }

    case PanelUtil::Mode::AUTO:
    {
        ui_->pushButton_Mode->setText(QString::fromStdString("Auto"));
        ui_->pushButton_Mode->setStyleSheet(
            "color: rgb(230, 19, 237);\nborder: 2px solid rgb(230, 19, 237);\nborder-radius: 15px");
        break;
    }

    default:
        break;
    }
}

void Server::Panel::on_ServerTab_currentChanged(int _tabIndex)
{
    ui_->ServerTab->setCurrentIndex(_tabIndex);

    switch (_tabIndex)
    {
    case PanelUtil::Tab::DASHBOARD:
    {
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::DASHBOARD, QColor(0, 255, 255));
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::ROBOT, Qt::white);
        break;
    }

    case PanelUtil::Tab::ROBOT:
    {
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::DASHBOARD, Qt::white);
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::ROBOT, QColor(0, 255, 255));
        break;
    }

    default:
        break;
    }
}

void Server::Panel::on_Start_clicked()
{
    if (not(planState_ == PanelUtil::PlanState::SUCCESS))
        return;

    planState_ = PanelUtil::PlanState::READY;

    msg_ = PanelUtil::Request::START_REQUEST;
    notify();
}

void Server::Panel::on_Stop_clicked()
{
    std_msgs::msg::Bool stopActivated;
    stopActivated.data = true;

    emergencyStop_->publish(stopActivated);
}

void Server::Panel::on_Scan_clicked()
{
    std_msgs::msg::Bool scanActivated;
    scanActivated.data = true;

    serverScan_->publish(scanActivated);
}

void Server::Panel::on_Plan_clicked()
{
    if (buttons_.size() == 0)
        return;

    switch (planState_)
    {
    case PanelUtil::PlanState::READY:
    {
        planState_ = PanelUtil::PlanState::PLANNING;

        msg_ = PanelUtil::Request::PLAN_REQUEST;
        notify();
        break;
    }

    case PanelUtil::PlanState::SUCCESS:
    {
        planState_ = PanelUtil::PlanState::READY;
        break;
    }

    case PanelUtil::PlanState::FAIL:
    {
        planState_ = PanelUtil::PlanState::READY;
        break;
    }

    default:
        break;
    }
}

void Server::Panel::on_pushButton_Mode_clicked()
{
    if (activatedRobotModeState_ == PanelUtil::Mode::MANUAL)
        instance_manager_->request_modeChange(activatedRobot_, PanelUtil::Mode::REMOTE);
    else
        instance_manager_->request_modeChange(activatedRobot_, PanelUtil::Mode::MANUAL);
}

void Server::Panel::on_pushButton_Kill_clicked()
{
    instance_manager_->request_kill(activatedRobot_);

    if (buttons_.contains(activatedRobot_))
    {
        instance_manager_->deleteRobot(activatedRobot_);
        deleteRobotButton(QString::fromStdString(activatedRobot_));
    }
}

void Server::Panel::keyPressEvent(QKeyEvent *_event)
{
    if (not(ui_->ServerTab->currentIndex() == PanelUtil::ROBOT))
        return;

    if (_event->key() == Qt::Key_Return)
    {
        geometry_msgs::msg::Pose2D goal;
        {
            goal.x = ui_->doubleSpinBox_goalX->value();
            goal.y = ui_->doubleSpinBox_goalY->value();
            goal.theta = ui_->doubleSpinBox_goalYaw->value();
        }

        instance_manager_->setGoal(activatedRobot_, goal);
    }

    if (activatedRobotModeState_ == PanelUtil::Mode::REMOTE)
    {
        switch (_event->key())
        {
        case Qt::Key_Up:
            activatedRobotLinVel_ = activatedRobotLinVel_ + 0.1;
            break;

        case Qt::Key_Down:
            activatedRobotLinVel_ = activatedRobotLinVel_ - 0.1;
            break;

        case Qt::Key_Left:
            activatedRobotAngVel_ = activatedRobotAngVel_ + 0.1;
            break;

        case Qt::Key_Right:
            activatedRobotAngVel_ = activatedRobotAngVel_ - 0.1;
            break;

        case Qt::Key_Space:
            activatedRobotLinVel_ = 0.0;
            activatedRobotAngVel_ = 0.0;
            break;

        default:
            break;
        }
    }
}

void Server::Panel::robotListDisp()
{
    if (buttons_.contains(inactivatedRobot_))
    {
        buttons_[inactivatedRobot_]->setStyleSheet(
            "color: white;\nborder: 2px solid white;\nborder-radius: 15px;");
    }

    if (buttons_.contains(activatedRobot_))
    {
        buttons_[activatedRobot_]->setStyleSheet(
            "color: rgb(255, 94, 0);\nborder: 3.5px solid rgb(255, 94, 0);\nborder-radius: 15px;");
    }
}

void Server::Panel::robotNumDisp()
{
    if (ui_->ServerTab->currentIndex() == PanelUtil::Tab::DASHBOARD)
        ui_->label_RobotNum->setText(QString::number(buttons_.size()));
}

void Server::Panel::robotTabDisp()
{
    if (ui_->ServerTab->currentIndex() == PanelUtil::Tab::ROBOT)
    {
        ui_->label_activated_robotName->setText(QString::fromStdString(activatedRobot_));

        QString lin_vel_qstring = QString::number(std::round(activatedRobotLinVel_ * 100.0) / 100.0);
        QString ang_vel_qstring = QString::number(std::round(activatedRobotAngVel_ * 100.0) / 100.0);

        ui_->label_Linear_Velocity->setText(lin_vel_qstring);
        ui_->label_Angular_Velocity->setText(ang_vel_qstring);
    }
}

void Server::Panel::modeButtonDisp()
{
    if (not(ui_->ServerTab->currentIndex() == PanelUtil::Tab::ROBOT))
        return;

    switch (activatedRobotModeState_)
    {
    case PanelUtil::Mode::MANUAL:
    {
        ui_->pushButton_Mode->setText(QString::fromStdString("Manual"));
        ui_->pushButton_Mode->setStyleSheet(
            "color: rgb(255,190,11);\nborder: 2px solid rgb(255,190,11);\nborder-radius: 15px");
        break;
    }

    case PanelUtil::Mode::REMOTE:
    {
        ui_->pushButton_Mode->setText(QString::fromStdString("Remote"));
        ui_->pushButton_Mode->setStyleSheet(
            "color: rgb(0,213,255);\nborder: 2px solid rgb(0,213,255);\nborder-radius: 15px");
        break;
    }

    case PanelUtil::Mode::AUTO:
    {
        ui_->pushButton_Mode->setText(QString::fromStdString("Auto"));
        ui_->pushButton_Mode->setStyleSheet(
            "color: rgb(230, 19, 237);\nborder: 2px solid rgb(230, 19, 237);\nborder-radius: 15px");
        break;
    }

    default:
        break;
    }
}

void Server::Panel::planButtonDisp()
{
    switch (planState_)
    {
    case PanelUtil::PlanState::READY:
        ui_->Plan->setText("Plan");
        ui_->Plan->setStyleSheet(
            "color: rgb(58, 134, 255);\nborder: 2px solid rgb(58, 134, 255);\nborder-radius: 15px;");
        break;

    case PanelUtil::PlanState::PLANNING:
        ui_->Plan->setText("Planning");
        ui_->Plan->setStyleSheet(
            "color: rgb(52, 235, 235);\nborder: 2px solid rgb(52, 235, 235);\nborder-radius: 15px;");
        break;

    case PanelUtil::PlanState::SUCCESS:
        ui_->Plan->setText("Success");
        ui_->Plan->setStyleSheet(
            "color: rgb(0, 200, 0);\nborder: 2px solid rgb(0, 200, 0);\nborder-radius: 15px;");
        break;

    case PanelUtil::PlanState::FAIL:
        ui_->Plan->setText("Fail");
        ui_->Plan->setStyleSheet(
            "color: rgb(200, 0, 0);\nborder: 2px solid rgb(200, 0, 0);\nborder-radius: 15px;");
        break;

    default:
        break;
    }
}

void Server::Panel::addRobotButton(QString _robotName)
{
    QPushButton *button = new QPushButton(_robotName);
    button->setObjectName(_robotName);

    ui_->scrollAreaWidgetContents->setFixedHeight(
        ui_->scrollAreaWidgetContents->height() + deltaScrollHeight_);

    button->setMinimumHeight(buttonHeight_);
    button->setFont(QFont("Ubuntu", buttonFontSize_, QFont::Bold));
    button->setStyleSheet(
        "color: white;\nborder: 2px solid white;\nborder-radius: 15px;");

    buttons_.insert(std::make_pair(_robotName.toStdString(), button));
    connect(button, SIGNAL(clicked()), this, SLOT(handleButton()));

    ui_->scrollArea_Robot_List->widget()->layout()->addWidget(button);
}

void Server::Panel::deleteRobotButton(QString _robotName)
{
    auto button = buttons_[_robotName.toStdString()];

    buttons_.erase(_robotName.toStdString());

    ui_->scrollAreaWidgetContents->setFixedHeight(
        ui_->scrollAreaWidgetContents->height() - deltaScrollHeight_);

    if (_robotName.toStdString() == activatedRobot_)
    {
        activatedRobot_ = std::string();
        ui_->ServerTab->setTabEnabled(PanelUtil::Tab::ROBOT, false);

        if (ui_->ServerTab->currentIndex() == PanelUtil::Tab::ROBOT)
            ui_->ServerTab->setCurrentIndex(PanelUtil::Tab::DASHBOARD);
    }

    delete button;
}

void Server::Panel::setPlanState(PanelUtil::PlanState _planState)
{
    if (not(_planState == PanelUtil::PlanState::SUCCESS or
            _planState == PanelUtil::PlanState::FAIL))
        return;

    planState_ = _planState;
}

// See: https://dev.to/fmtweisszwerg/cc-how-to-get-all-interface-addresses-on-the-local-device-3pki
std::string Server::Panel::getIPAddress()
{
    struct ifaddrs *ptr_ifaddrs = nullptr;

    auto result = getifaddrs(&ptr_ifaddrs);

    // getifaddrs() failed.
    if (result != 0)
        return "000.000.000.000";

    std::string ipaddress_human_readable_form;

    for (
        struct ifaddrs *ptr_entry = ptr_ifaddrs;
        ptr_entry != nullptr;
        ptr_entry = ptr_entry->ifa_next)
    {

        std::string interface_name = std::string(ptr_entry->ifa_name);
        sa_family_t address_family = ptr_entry->ifa_addr->sa_family;

        if (address_family == AF_INET)
        {
            // IPv4

            // Be aware that the `ifa_addr`, `ifa_netmask` and `ifa_data` fields might contain nullptr.
            // Dereferencing nullptr causes "Undefined behavior" problems.
            // So it is need to check these fields before dereferencing.
            if (ptr_entry->ifa_addr != nullptr)
            {
                char buffer[INET_ADDRSTRLEN] = {
                    0,
                };
                inet_ntop(
                    address_family,
                    &((struct sockaddr_in *)(ptr_entry->ifa_addr))->sin_addr,
                    buffer,
                    INET_ADDRSTRLEN);

                ipaddress_human_readable_form = std::string(buffer);
            }

            if (interface_name != "lo")
                break;
        }
        else if (address_family == AF_INET6)
        {
            // IPv6
            if (ptr_entry->ifa_addr != nullptr)
            {
                char buffer[INET6_ADDRSTRLEN] = {
                    0,
                };
                inet_ntop(
                    address_family,
                    &((struct sockaddr_in6 *)(ptr_entry->ifa_addr))->sin6_addr,
                    buffer,
                    INET6_ADDRSTRLEN);

                ipaddress_human_readable_form = std::string(buffer);
            }

            if (interface_name != "lo")
                break;
        }
        else
        {
            // AF_UNIX, AF_UNSPEC, AF_PACKET etc.
            // If ignored, delete this section.
        }
    }

    freeifaddrs(ptr_ifaddrs);

    return ipaddress_human_readable_form;
}

void Server::Panel::register_robot(
    const std::shared_ptr<PanelUtil::Connection::Request> _request,
    std::shared_ptr<PanelUtil::Connection::Response> _response)
{
    if (buttons_.contains(_request->config.name))
    {
        _response->is_connected = true;
        return;
    }

    static int32_t robotID = 0;

    AgentInstance::Agent robotInfo;
    {
        robotInfo.name_ = _request->config.name;

        robotInfo.type_ = _request->config.type;
        robotInfo.size_ = _request->config.size;
        robotInfo.wheel_radius_ = _request->config.wheel_radius;
        robotInfo.wheel_seperation_ = _request->config.wheel_seperation;

        robotInfo.max_linVel_ = _request->config.max_linvel;
        robotInfo.max_linAcc_ = _request->config.max_linacc;
        robotInfo.max_angVel_ = _request->config.max_angvel;
        robotInfo.max_angAcc_ = _request->config.max_angacc;

        robotInfo.goal_.component_ = _request->goal;
    }

    AgentInstance::Robot robot;
    {
        robot.robotInfo_ = robotInfo;
        robot.id_ = robotID;
        robot.mode_ = PanelUtil::Mode::MANUAL;

        robot.prior_update_time_ = nh_->now();
        robot.last_update_time_ = nh_->now();

        robot.modeFromRobot_ = nh_->create_service<PanelUtil::ModeSelection>(
            "/" + robotInfo.name_ + "/modeFromRobot",
            std::bind(&Server::Panel::change_robot_mode, this, std::placeholders::_1, std::placeholders::_2));
        robot.modeFromServer_ = nh_->create_client<PanelUtil::ModeSelection>("/" + robotInfo.name_ + "/modeFromServer");
    }

    instance_manager_->insertRobot(robot);
    emit addRobotSignal(QString::fromStdString(_request->config.name));

    _response->is_connected = true;
}

void Server::Panel::expire_robot(
    const std::shared_ptr<PanelUtil::Disconnection::Request> _request,
    std::shared_ptr<PanelUtil::Disconnection::Response> _response)
{
    if (buttons_.contains(_request->name))
    {
        instance_manager_->deleteRobot(_request->name);
        deleteRobotButton(QString::fromStdString(_request->name));
    }
    _response->is_disconnected = true;
}

void Server::Panel::change_robot_mode(
    const std::shared_ptr<PanelUtil::ModeSelection::Request> _request,
    std::shared_ptr<PanelUtil::ModeSelection::Response> _response)
{
    if (_request->is_remote == true)
        instance_manager_->setMode(_request->name, PanelUtil::Mode::REMOTE);
    else
        instance_manager_->setMode(_request->name, PanelUtil::Mode::MANUAL);

    _response->is_complete = true;
}

void Server::Panel::update()
{
    update_robot_tab();
    update_rviz_poseArray();
}

void Server::Panel::update_robot_tab()
{
    if (not(ui_->ServerTab->currentIndex() == PanelUtil::Tab::ROBOT))
        return;

    auto robot = instance_manager_->getRobot(activatedRobot_);

    activatedRobotModeState_ = robot.mode_;
    if (activatedRobotModeState_ == PanelUtil::Mode::REMOTE)
    {
        geometry_msgs::msg::Twist remote_cmd_vel;
        {
            remote_cmd_vel.linear.x = activatedRobotLinVel_;
            remote_cmd_vel.angular.z = activatedRobotAngVel_;
        }
        instance_manager_->remote_control(activatedRobot_, remote_cmd_vel);
    }
    else
    {
        activatedRobotLinVel_ = robot.robotInfo_.linVel_;
        activatedRobotAngVel_ = robot.robotInfo_.angVel_;
    }
}

void Server::Panel::update_rviz_poseArray()
{
    if (buttons_.empty())
        return;

    visualization_msgs::msg::MarkerArray robotPoseMarkerArray;
    {
        robotPoseMarkerArray.markers.clear();

        for (const auto &buttonPair : buttons_)
        {
            std::string robotName = buttonPair.first;

            auto &robot = instance_manager_->getRobot(robotName);

            if (robot.prior_update_time_.seconds() < 1e-8)
                continue;

            auto robotPoseMarker = make_robotPoseMarker(robot);

            if (robotPoseMarker.ns == "")
                continue;

            robotPoseMarkerArray.markers.push_back(robotPoseMarker);
        }
    }

    rviz_poses_pub_->publish(robotPoseMarkerArray);
}

visualization_msgs::msg::Marker Server::Panel::make_robotPoseMarker(const AgentInstance::Robot &_robot)
{
    auto robotPoseMarker = visualization_msgs::msg::Marker();

    robotPoseMarker.header.frame_id = "map";
    robotPoseMarker.header.stamp = _robot.last_update_time_;
    robotPoseMarker.ns = _robot.robotInfo_.name_;
    robotPoseMarker.id = _robot.id_;
    robotPoseMarker.type = visualization_msgs::msg::Marker::ARROW;
    robotPoseMarker.action = visualization_msgs::msg::Marker::ADD;

    robotPoseMarker.scale.x = 0.5;
    robotPoseMarker.scale.y = 0.125;
    robotPoseMarker.scale.z = 0.125;

    if (activatedRobot_ == _robot.robotInfo_.name_)
    {
        robotPoseMarker.color.r = 1.0;
        robotPoseMarker.color.g = 0.5;
        robotPoseMarker.color.b = 0.5;
        robotPoseMarker.color.a = 1.0;
    }
    else
    {
        robotPoseMarker.color.r = 0.5;
        robotPoseMarker.color.g = 0.5;
        robotPoseMarker.color.b = 1.0;
        robotPoseMarker.color.a = 1.0;
    }

    robotPoseMarker.lifetime = _robot.last_update_time_ - _robot.prior_update_time_;

    robotPoseMarker.pose.position.x = _robot.robotInfo_.pose_.component_.x;
    robotPoseMarker.pose.position.y = _robot.robotInfo_.pose_.component_.y;
    robotPoseMarker.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, _robot.robotInfo_.pose_.component_.theta);
    robotPoseMarker.pose.orientation.x = q.x();
    robotPoseMarker.pose.orientation.y = q.y();
    robotPoseMarker.pose.orientation.z = q.z();
    robotPoseMarker.pose.orientation.w = q.w();

    return robotPoseMarker;
}

Server::Panel::Panel(
    std::shared_ptr<rclcpp::Node> &_nh,
    std::shared_ptr<Instance_Manager> _instance_manager,
    QWidget *_parent)
    : QWidget(_parent), ui_(new Ui::ServerPanel),
      nh_(_nh), instance_manager_(_instance_manager)
{
    ui_->setupUi(this);

    setFocusPolicy(Qt::StrongFocus);

    setWindowFlags(
        Qt::WindowStaysOnTopHint |
        Qt::Window |
        Qt::WindowTitleHint |
        Qt::CustomizeWindowHint |
        Qt::WindowMinimizeButtonHint);

    msg_ = PanelUtil::Request::NO_REQUEST;

    ui_->label_IPAddress->setText(QString::fromStdString(getIPAddress()));
    planState_ = PanelUtil::PlanState::READY;

    buttons_.clear();
    ui_->scrollAreaWidgetContents->setFixedHeight(0);
    ui_->scrollArea_Robot_List->widget()->setLayout(new QVBoxLayout());
    ui_->scrollArea_Robot_List->widget()->layout()->setSpacing(scrollSpacing_);
    ui_->scrollArea_Robot_List->setWidgetResizable(true);

    ui_->ServerTab->currentChanged(PanelUtil::Tab::DASHBOARD);
    ui_->ServerTab->setTabEnabled(PanelUtil::Tab::ROBOT, false);

    displayTimer_ = new QTimer(this);
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(robotListDisp()));
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(robotNumDisp()));
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(robotTabDisp()));
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(modeButtonDisp()));
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(planButtonDisp()));

    connect(this, SIGNAL(addRobotSignal(QString)), this, SLOT(addRobotButton(QString)));

    displayTimer_->start(10);

    // init ROS2 Instances
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    connection_ = nh_->create_service<PanelUtil::Connection>(
        "/connection",
        std::bind(&Server::Panel::register_robot, this, std::placeholders::_1, std::placeholders::_2));
    disconnection_ = nh_->create_service<PanelUtil::Disconnection>(
        "/disconnection",
        std::bind(&Server::Panel::expire_robot, this, std::placeholders::_1, std::placeholders::_2));

    serverScan_ = nh_->create_publisher<std_msgs::msg::Bool>("/server_scan", qos);
    emergencyStop_ = nh_->create_publisher<std_msgs::msg::Bool>("/emergency_stop", qos);

    rviz_poses_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("robot_list", qos);

    update_timer_ = nh_->create_wall_timer(
        10ms, std::bind(&Server::Panel::update, this));
}

Server::Panel::~Panel()
{
    delete ui_;
}