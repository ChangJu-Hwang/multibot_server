#include "multibot_server/server_panel.hpp"

#include <iostream>

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

#include "multibot_server/ui_server_panel.h"

void Server::Panel::handleButton()
{
    QObject *button = QObject::sender();

    inactivatedRobot_ = activatedRobot_;
    activatedRobot_ = button->objectName().toStdString();

    ui_->ServerTab->currentChanged(PanelUtil::Tab::ROBOT);
}

void Server::Panel::on_ServerTab_currentChanged(int _tabIndex)
{
    ui_->ServerTab->setCurrentIndex(_tabIndex);

    switch (_tabIndex)
    {
    case PanelUtil::Tab::DASHBOARD:
    {
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::DASHBOARD, QColor(0, 255, 255));
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::TASKS, Qt::white);
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::ROBOT, Qt::white);
        break;
    }

    case PanelUtil::Tab::TASKS:
    {
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::DASHBOARD, Qt::white);
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::TASKS, QColor(0, 255, 255));
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::ROBOT, Qt::white);
        break;
    }

    case PanelUtil::Tab::ROBOT:
    {
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::DASHBOARD, Qt::white);
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::TASKS, Qt::white);
        ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::ROBOT, QColor(0, 255, 255));
        break;
    }

    default:
        break;
    }
}

void Server::Panel::on_Start_clicked()
{
    std::cout << "Button: Start clicked" << std::endl;
}

void Server::Panel::on_Stop_clicked()
{
    std::cout << "Button: Stop clicked" << std::endl;
}

void Server::Panel::on_Scan_clicked()
{
    std::cout << "Button: Scan clicked" << std::endl;
}

void Server::Panel::on_Reset_clicked()
{
    std::cout << "Button: Reset clicked" << std::endl;
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
    ui_->label_RobotNum->setText(QString::number(buttons_.size()));
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
                char buffer[INET_ADDRSTRLEN] = {0,};
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
                char buffer[INET6_ADDRSTRLEN] = {0,};
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

Server::Panel::Panel(QWidget *_parent)
    : QWidget(_parent), ui_(new Ui::ServerPanel)
{
    ui_->setupUi(this);

    setWindowFlags(
        Qt::WindowStaysOnTopHint |
        Qt::Window |
        Qt::WindowTitleHint |
        Qt::CustomizeWindowHint |
        Qt::WindowMinimizeButtonHint);

    ui_->label_IPAddress->setText(QString::fromStdString(getIPAddress()));

    buttons_.clear();
    ui_->scrollAreaWidgetContents->setFixedHeight(0);
    ui_->scrollArea_Robot_List->widget()->setLayout(new QVBoxLayout());
    ui_->scrollArea_Robot_List->widget()->layout()->setSpacing(scrollSpacing_);
    ui_->scrollArea_Robot_List->setWidgetResizable(true);

    ui_->ServerTab->currentChanged(PanelUtil::Tab::DASHBOARD);

    displayTimer_ = new QTimer(this);
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(robotListDisp()));
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(robotNumDisp()));
    connect(this, SIGNAL(addRobotSignal(QString)), this, SLOT(addRobotButton(QString)));
    connect(this, SIGNAL(deleteRobotSignal(QString)), this, SLOT(deleteRobotButton(QString)));

    displayTimer_->start(10);
}

Server::Panel::~Panel()
{
    delete ui_;
}