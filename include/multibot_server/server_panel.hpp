#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <QWidget>
#include <QTimer>
#include <QPushButton>

#include "multibot_util/Panel_Util.hpp"
#include "multibot_util/Interface/Observer_Interface.hpp"

#include "multibot_server/Instance_Manager.hpp"


using namespace Instance;
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
        void on_Plan_clicked();
        void on_pushButton_Mode_clicked();
        void on_pushButton_Kill_clicked();

        void keyPressEvent(QKeyEvent *_event) override;

    private slots:
        void robotListDisp();
        void robotNumDisp();
        void robotTabDisp();
        void modeButtonDisp();
        void planButtonDisp();

    private slots:
        void addRobotButton(QString _robotName);
        void deleteRobotButton(QString _robotName);

    public:
        void setPlanState(PanelUtil::PlanState _planState);

    private:
        std::string getIPAddress();

    private:
        Ui::ServerPanel *ui_;
        QTimer *displayTimer_;

    private:
        void register_robot(
            const std::shared_ptr<PanelUtil::Connection::Request> _request,
            std::shared_ptr<PanelUtil::Connection::Response> _response);

        void expire_robot(
            const std::shared_ptr<PanelUtil::Disconnection::Request> _request,
            std::shared_ptr<PanelUtil::Disconnection::Response> _response);

        void change_robot_mode(
            const std::shared_ptr<PanelUtil::ModeSelection::Request> _request,
            std::shared_ptr<PanelUtil::ModeSelection::Response> _response);

        void update();
        void update_robot_tab();
        void update_rviz_poseArray();

        visualization_msgs::msg::Marker make_robotPoseMarker(const AgentInstance::Robot &_robot);

    private:
        std::shared_ptr<rclcpp::Node> nh_;
        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Service<PanelUtil::Connection>::SharedPtr connection_;
        rclcpp::Service<PanelUtil::Disconnection>::SharedPtr disconnection_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr serverScan_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergencyStop_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_poses_pub_;

    private:
        std::unordered_map<std::string, QPushButton *> buttons_;

        std::string activatedRobot_;
        std::string inactivatedRobot_;

        double activatedRobotLinVel_;
        double activatedRobotAngVel_;
        geometry_msgs::msg::Pose2D activatedRobotGoal_;
        PanelUtil::Mode activatedRobotModeState_;
        PanelUtil::PlanState planState_;

        static constexpr int scrollSpacing_ = 20;
        static constexpr int buttonHeight_ = 50;
        static constexpr int buttonFontSize_ = 13;
        static constexpr int deltaScrollHeight_ = 70;

        PanelUtil::Msg msg_;
        std::list<Observer::ObserverInterface<PanelUtil::Msg> *> list_observer_;

        std::shared_ptr<Instance_Manager> instance_manager_;

    public:
        Panel(
            std::shared_ptr<rclcpp::Node> &_nh,
            std::shared_ptr<Instance_Manager> _instance_manager,
            QWidget *_parent = nullptr);
        ~Panel();
    }; // class Panel
} // namespace Server