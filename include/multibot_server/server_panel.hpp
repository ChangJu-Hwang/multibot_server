#pragma once

#include <QWidget>
#include <QTimer>
#include <QPushButton>

#include "multibot_util/Interface/Observer_Interface.hpp"

namespace PanelUtil
{
    enum Tab {DASHBOARD, TASKS, ROBOT};
} // namespace PanelUtil;

namespace Ui
{
    class ServerPanel;
} // namespace Ui

namespace Server
{
    class Panel : public QWidget
    {
    private:
        Q_OBJECT
    
    signals:
        void addRobotSignal(QString _robotName);
        void deleteRobotSignal(QString _robotName);

    private slots:
        void handleButton();
        void on_ServerTab_currentChanged(int _tabIndex);
        void on_Start_clicked();
        void on_Stop_clicked();
        void on_Scan_clicked();
        void on_Reset_clicked();

        void robotListDisp();
        void robotNumDisp();

        void addRobotButton(QString _robotName);
        void deleteRobotButton(QString _robotName);

    public:
        void addRobot(std::string _robotName);
        void deleteRobot(std::string _robotName);

    private:
        std::string getIPAddress();

    private:
        Ui::ServerPanel *ui_;
        QTimer *displayTimer_;

        std::unordered_map<std::string, QPushButton*> buttons_;

        std::string activatedRobot_;
        std::string inactivatedRobot_;

        static constexpr int scrollSpacing_ = 20;
        static constexpr int buttonHeight_ = 50;
        static constexpr int buttonFontSize_ = 13;
        static constexpr int deltaScrollHeight_ = 70;
    
    public:
        explicit Panel(QWidget *_parent = nullptr);
        ~Panel();
    }; // class Panel
} // namespace Server