#ifndef ROBOTDATAFETCHER_H
#define ROBOTDATAFETCHER_H

#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtCore/QThread>
#include <QtCore/QSharedPointer>
#include <array>
#include "RB/rbpodo.hpp" // 로봇 SDK 헤더

// mainwindow.cpp에서 정의된 타입들을 다시 사용
using CobotType = rb::podo::Cobot<rb::podo::StandardVector>;
using JointAngleType = std::array<double, 6>; 
using TCPPoseType = std::array<double, 6>; 

class RobotDataFetcher : public QObject
{
    Q_OBJECT

public:
    // 생성자: 로봇 포인터와 업데이트 주기를 받습니다.
    explicit RobotDataFetcher(QSharedPointer<CobotType> robotPtr, int intervalMs, QObject *parent = nullptr);
    ~RobotDataFetcher() override;

private:
    QSharedPointer<CobotType> robot;
    QTimer *pollingTimer;
    const int interval;
    
    // 시스템 변수 배열 (mainwindow.cpp에서 복사)
    constexpr static rb::podo::SystemVariable JOINT_ANGLE_VARS[] = {
        rb::podo::SystemVariable::SD_J0_ANG, rb::podo::SystemVariable::SD_J1_ANG, rb::podo::SystemVariable::SD_J2_ANG,
        rb::podo::SystemVariable::SD_J3_ANG, rb::podo::SystemVariable::SD_J4_ANG, rb::podo::SystemVariable::SD_J5_ANG
    };
    constexpr static rb::podo::SystemVariable TCP_POSE_VARS[] = {
        rb::podo::SystemVariable::SD_TCP_X, rb::podo::SystemVariable::SD_TCP_Y, rb::podo::SystemVariable::SD_TCP_Z,
        rb::podo::SystemVariable::SD_TCP_RX, rb::podo::SystemVariable::SD_TCP_RY, rb::podo::SystemVariable::SD_TCP_RZ
    };

private slots:
    // 💡 별도의 스레드에서 실행될 실제 데이터 요청 슬롯 (Blocking 코드 포함)
    void doPolling();
    
public slots:
    // 스레드가 시작될 때 호출될 슬롯
    void startPolling();
    // 스레드 외부에서 호출되어 안전하게 타이머를 멈추고 스레드를 종료시킴
    void stopPolling();

signals:
    // UI 업데이트를 위해 메인 스레드로 전송할 시그널
    void jointAnglesUpdated(const JointAngleType& angles);
    void tcpPoseUpdated(const TCPPoseType& pose);
    void pollingError(const QString& errorMsg);
    void finished(); // 스레드 종료 알림 시그널
};

#endif // ROBOTDATAFETCHER_H