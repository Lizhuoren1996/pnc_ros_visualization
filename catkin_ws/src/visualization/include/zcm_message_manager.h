#ifndef _PLANNING_ZCM_MESSAGE_MANAGER_H_
#define _PLANNING_ZCM_MESSAGE_MANAGER_H_
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <zcm/zcm-cpp.hpp>
#include <future>

#include "timestamp.h"
#include "MsgViewTrajectory.hpp"
#include "MsgViewReflineList.hpp"
#include "MsgViewObstacleList.hpp"
#include "MsgViewParkinglotList.hpp"
#include "MsgViewObstacleTrajectoryPoint.hpp"
#include "MsgViewLine.hpp"
#include "MsgViewObstacleList.hpp"
#include "MsgFusionMap.hpp"

namespace planning {

static std::mutex message_manager_mutex;

class Handler {
public:
    // various zcm message subscribers
    void HandleMsgViewStartSpot(const zcm::ReceiveBuffer *rbuf,
                            const std::string &chan, const MsgViewPoint *msg);

    void HandleMsgViewTrajectory(const zcm::ReceiveBuffer *rbuf,
                                const std::string &chan,
                                const MsgViewTrajectory *msg);
    
    void HandleMsgFusionMap(const zcm::ReceiveBuffer *rbuf,
                                const std::string &chan,
                                const MsgFusionMap *msg);
    
    void HandleMsgViewRefline(const zcm::ReceiveBuffer *rbuf,
                            const std::string &chan, const MsgViewRefline *msg);
    
    void HandleMsgViewReflineList(const zcm::ReceiveBuffer *rbuf,
                            const std::string &chan, const MsgViewReflineList *msg);
    
    void HandleMsgViewParkinglotLists(const zcm::ReceiveBuffer *rbuf,
                            const std::string &chan, const MsgViewParkinglotList *msg);

    void HandleMsgViewObstacleList(const zcm::ReceiveBuffer *rbuf,
                            const std::string &chan, const MsgViewObstacleList *msg);
  
  
    /* timestamps update when handling the external published messages */
    // time_t update_time_nav_info_signal;
    time_t update_time_fusion_map;

    /* flags indicate if the external published messages handled successfully */
    bool flag_received_startspot = false;
    bool flag_received_trajectory = false;
    bool flag_received_refline = false;
    bool flag_received_reflinelist = false;
    bool flag_received_parkinglotlist = false;
    bool flag_received_obstaclelist = false;
    bool flag_received_fusion_map = false;

    /* thread locks of handled messages */
    std::mutex startspot_mutex;
    std::mutex trajectory_mutex;
    std::mutex refline_mutex;
    std::mutex reflinelist_mutex;
    std::mutex parkinglotlist_mutex;
    std::mutex obstaclelist_mutex;
    std::mutex fusion_map_mutex;
    

    /* The handled messages are stored in these variables temporarily */
    MsgViewPoint tmp_startspot;
    MsgViewTrajectory tmp_trajectory;
    MsgViewRefline tmp_refline;
    MsgViewReflineList tmp_reflinelist;
    MsgViewParkinglotList tmp_parkinglotlist;
    MsgViewObstacleList tmp_obstaclelist;
    MsgFusionMap tmp_fusion_map;
};



class ZcmMessageManager {
public:

    

    static ZcmMessageManager *GetInstance() {
        message_manager_mutex.lock();
        static ZcmMessageManager instance;
        message_manager_mutex.unlock();
        return &instance;
    }

    const static time_t TIMEOUT_US_FUSION_MAP = 1e6;

    bool Start();
    void Stop();

    void ReceiveMessagesOnIpc();
    bool GetViewReflinelist(MsgViewReflineList &view_reflinelist);
    bool GetViewTrajectory(MsgViewTrajectory &view_trajectory);
    bool GetViewParkinglotlist(MsgViewParkinglotList &view_parkinglotlist);
    bool GetViewObstaclelist(MsgViewObstacleList &view_obstaclelist);
    bool GetViewStartSpot(MsgViewPoint &view_startspot);
    bool GetViewRefline(MsgViewRefline &view_refline);
    bool GetFusionMap(MsgFusionMap &fusion_map);



private:
    ZcmMessageManager() = default;
    ~ZcmMessageManager() = default;

    Handler inner_handler;
    /* zcm url */
    zcm::ZCM zcm_ipc{"ipc"};

    std::future<void> task_future_;
    std::atomic<bool> is_stop_{false};
};



} // namespace planning




#endif