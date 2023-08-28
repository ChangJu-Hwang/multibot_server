#pragma once

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <QWidget>
#include <QTimer>
#include <QPushButton>

#include "multibot_util/Interface/Observer_Interface.hpp"

namespace PanelUtil
{
    enum Tab
    {
        DASHBOARD,
        TASKS,
        ROBOT
    }; // enum Tab

    enum Request
    {
        NO_REQUEST,
        SET_GOAL,
        SET_TARGET,
        SCAN,
        KILL,
        REMOTE_REQUEST,
        MANUAL_REQUEST
    }; // enum Request

    enum Mode
    {
        REMOTE,
        MANUAL,
        AUTO
    }; // enum Mode

    typedef std::tuple<Request, std::string, geometry_msgs::msg::Pose2D> Msg;
} // namespace PanelUtil;

namespace Ui
{
    class ServerPanel;
} // namespace Ui

namespace Server
{
    class Panel : public QWidget, public Observer::SubjectInterface<PanelUtil::Msg>
    {
    private:
        Q_OBJECT

    signals:
        void addRobotSignal(QString _robotName);
        void deleteRobotSignal(QString _robotName);

    public:
        void attach(Observer::ObserverInterface<PanelUtil::Msg> &_observer) override;
        void detach(Observer::ObserverInterface<PanelUtil::Msg> &_observer) override;
        void notify() override;

    private slots:
        void handleButton();
        void on_ServerTab_currentChanged(int _tabIndex);
        void on_Start_clicked();
        void on_Stop_clicked();
        void on_Scan_clicked();
        void on_pushButton_Mode_clicked();
        void on_pushButton_Kill_clicked();

        void keyPressEvent(QKeyEvent *_event) override;

    private slots:
        void robotListDisp();
        void robotNumDisp();
        void robotTabDisp();
        void modeButtonDisp();

    private slots:
        void addRobotButton(QString _robotName);
        void deleteRobotButton(QString _robotName);

    public:
        void addRobot(std::string _robotName);
        void deleteRobot(std::string _robotName);
        void storeGoal(geometry_msgs::msg::Pose2D _goal);
        void setVelocity(double _linVel, double _angVel);
        void setModeState(PanelUtil::Mode _mode);
        
        PanelUtil::Mode getModeState() { return activatedRobotModeState_; }
        int getCurrentTabIndex();
        geometry_msgs::msg::Twist get_cmd_vel();

    private:
        std::string getIPAddress();

    private:
        Ui::ServerPanel *ui_;
        QTimer *displayTimer_;

        std::unordered_map<std::string, QPushButton *> buttons_;

        std::string activatedRobot_;
        std::string inactivatedRobot_;

        double activatedRobotLinVel_;
        double activatedRobotAngVel_;
        geometry_msgs::msg::Pose2D activatedRobotGoal_;
        PanelUtil::Mode activatedRobotModeState_;

        static constexpr int scrollSpacing_ = 20;
        static constexpr int buttonHeight_ = 50;
        static constexpr int buttonFontSize_ = 13;
        static constexpr int deltaScrollHeight_ = 70;

        PanelUtil::Msg msg_;
        std::list<Observer::ObserverInterface<PanelUtil::Msg> *> list_observer_;

    public:
        explicit Panel(QWidget *_parent = nullptr);
        ~Panel();
    }; // class Panel
} // namespace Server