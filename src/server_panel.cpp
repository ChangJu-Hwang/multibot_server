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

    std::get<0>(msg_) = PanelUtil::Request::SET_TARGET;
    std::get<1>(msg_) = activatedRobot_;
    notify();

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

    std::get<0>(msg_) = PanelUtil::Request::START_REQUEST;
    notify();
}

void Server::Panel::on_Stop_clicked()
{
    std::get<0>(msg_) = PanelUtil::Request::STOP_REQUEST;
    notify();
}

void Server::Panel::on_Scan_clicked()
{
    std::get<0>(msg_) = PanelUtil::Request::SCAN_REQUEST;
    notify();
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

        std::get<0>(msg_) = PanelUtil::Request::PLAN_REQUEST;
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
    {
        std::get<0>(msg_) = PanelUtil::Request::REMOTE_REQUEST;
        std::get<1>(msg_) = activatedRobot_;

        notify();
    }
    else
    {
        std::get<0>(msg_) = PanelUtil::Request::MANUAL_REQUEST;
        std::get<1>(msg_) = activatedRobot_;

        notify();
    }
}

void Server::Panel::on_pushButton_Kill_clicked()
{
    std::get<0>(msg_) = PanelUtil::Request::KILL_REQUEST;
    std::get<1>(msg_) = activatedRobot_;

    notify();
}

void Server::Panel::keyPressEvent(QKeyEvent *_event)
{
    if (not(ui_->ServerTab->currentIndex() == PanelUtil::ROBOT))
        return;

    if (_event->key() == Qt::Key_Return)
    {
        std::get<0>(msg_) = PanelUtil::SET_GOAL;
        std::get<1>(msg_) = activatedRobot_;

        geometry_msgs::msg::Pose2D goal;
        {
            goal.x = ui_->doubleSpinBox_goalX->value();
            goal.y = ui_->doubleSpinBox_goalY->value();
            goal.theta = ui_->doubleSpinBox_goalYaw->value();
        }
        std::get<2>(msg_) = goal;

        notify();
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

void Server::Panel::addRobot(std::string _robotName)
{
    emit addRobotSignal(QString::fromStdString(_robotName));
}

void Server::Panel::deleteRobot(std::string _robotName)
{
    emit deleteRobotSignal(QString::fromStdString(_robotName));
}

void Server::Panel::storeGoal(geometry_msgs::msg::Pose2D _goal)
{
    activatedRobotGoal_ = _goal;
}

void Server::Panel::setVelocity(double _linVel, double _angVel)
{
    activatedRobotLinVel_ = _linVel;
    activatedRobotAngVel_ = _angVel;
}

void Server::Panel::setModeState(PanelUtil::Mode _mode)
{
    activatedRobotModeState_ = _mode;
}

void Server::Panel::setPlanState(PanelUtil::PlanState _planState)
{
    if (not(planState_ == PanelUtil::PlanState::PLANNING))
        return;

    if (not(_planState == PanelUtil::PlanState::SUCCESS or
            _planState == PanelUtil::PlanState::FAIL))
        return;

    planState_ = _planState;
}

int Server::Panel::getCurrentTabIndex()
{
    return (ui_->ServerTab->currentIndex());
}

geometry_msgs::msg::Twist Server::Panel::get_cmd_vel()
{
    geometry_msgs::msg::Twist cmd_vel;

    cmd_vel.linear.x = activatedRobotLinVel_;
    cmd_vel.angular.z = activatedRobotAngVel_;

    return cmd_vel;
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

void Server::Panel::lockUi()
{
    ui_->ServerTab->setTabEnabled(PanelUtil::Tab::ROBOT, false);

    ui_->Start->setEnabled(false);
    ui_->Plan->setEnabled(false);
    ui_->Start->setEnabled(false);
    ui_->Stop->setEnabled(false);
    ui_->pushButton_Mode->setEnabled(false);
    ui_->pushButton_Kill->setEnabled(false);
}

void Server::Panel::unlockUi()
{
    ui_->ServerTab->setTabEnabled(PanelUtil::Tab::ROBOT, true);

    ui_->Start->setEnabled(true);
    ui_->Plan->setEnabled(true);
    ui_->Start->setEnabled(true);
    ui_->Stop->setEnabled(true);
    ui_->pushButton_Mode->setEnabled(true);
    ui_->pushButton_Kill->setEnabled(true);
}

Server::Panel::Panel(QWidget *_parent)
    : QWidget(_parent), ui_(new Ui::ServerPanel)
{
    ui_->setupUi(this);

    setFocusPolicy(Qt::StrongFocus);

    setWindowFlags(
        Qt::WindowStaysOnTopHint |
        Qt::Window |
        Qt::WindowTitleHint |
        Qt::CustomizeWindowHint |
        Qt::WindowMinimizeButtonHint);

    std::get<0>(msg_) = PanelUtil::Request::NO_REQUEST;
    std::get<1>(msg_) = std::string();
    std::get<2>(msg_) = geometry_msgs::msg::Pose2D();

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
    connect(this, SIGNAL(deleteRobotSignal(QString)), this, SLOT(deleteRobotButton(QString)));

    displayTimer_->start(10);
}

Server::Panel::~Panel()
{
    delete ui_;
}