#include "zcm_message_manager.h"
#include "timestamp.h"

using namespace std;

namespace planning {


//zcm_viewstartspot callback()
void Handler::HandleMsgViewStartSpot(const zcm::ReceiveBuffer *rbuf,
                            const std::string &chan, const MsgViewPoint *msg){
    startspot_mutex.lock();
    tmp_startspot = *msg;
    flag_received_startspot = true;
    // std::cout<< "receive startspot" <<std::endl;
    startspot_mutex.unlock();
}

void Handler::HandleMsgFusionMap(const zcm::ReceiveBuffer *rbuf,
                            const std::string &chan, const MsgFusionMap *msg){
    fusion_map_mutex.lock();
    tmp_fusion_map = *msg;
    // std::cout<< "**********"<<msg->map_cells.size()<<std::endl;
    flag_received_fusion_map = true;
    // update_time_fusion_map = true;
    // std::cout<< "receive fusion map" <<std::endl; 

    fusion_map_mutex.unlock();
}

void Handler::HandleMsgViewRefline(const zcm::ReceiveBuffer *rbuf,
                            const std::string &chan, const MsgViewRefline *msg){
    refline_mutex.lock();
    tmp_refline = *msg;
    flag_received_refline = true;
    // std::cout << "receive refline" << std::endl;
    refline_mutex.unlock();
}

//zcm_viewreflines callback()
void Handler::HandleMsgViewReflineList(const zcm::ReceiveBuffer *rbuf,
                            const std::string &chan, const MsgViewReflineList *msg){
    reflinelist_mutex.lock();
    tmp_reflinelist = *msg;
    flag_received_reflinelist = true;
    // std::cout << "receive reflines" << std::endl;
    reflinelist_mutex.unlock();
}

// zcm_viewtrajectory callback()
void Handler::HandleMsgViewTrajectory(const zcm::ReceiveBuffer *rbuf,
                            const std::string &chan,
                            const MsgViewTrajectory *msg) {
    trajectory_mutex.lock();
    tmp_trajectory = *msg; //get zcm_msg
    flag_received_trajectory = true;
    // update_time_nav_info_signal = getTimeStamp();
    // std::cout << "receive trajectory" << std::endl;
    trajectory_mutex.unlock();
}

// zcm_viewparkinglots callback()
void Handler::HandleMsgViewParkinglotLists(const zcm::ReceiveBuffer *rbuf,
                            const std::string &chan, const MsgViewParkinglotList *msg){
    parkinglotlist_mutex.lock();
    tmp_parkinglotlist = *msg;
    flag_received_parkinglotlist = true;
    // std::cout<< "receive parkinglotlist" <<std::endl;
    parkinglotlist_mutex.unlock();

}


// zcm_viewobstacle callback()
void Handler::HandleMsgViewObstacleList(const zcm::ReceiveBuffer *rbuf,
                        const std::string &chan, const MsgViewObstacleList *msg){
    obstaclelist_mutex.lock();
    tmp_obstaclelist = *msg;
    flag_received_obstaclelist = true;
    // std::cout<< "receive obstaclelist" <<std::endl;
    obstaclelist_mutex.unlock();            
}




bool ZcmMessageManager::Start() {
    is_stop_ = false;
    task_future_ = std::async(std::launch::async, &ZcmMessageManager::ReceiveMessagesOnIpc, this);
    return true;
}

void ZcmMessageManager::Stop() {
    is_stop_ = true;
    task_future_.get();
}

// zcm message subscribers
void ZcmMessageManager::ReceiveMessagesOnIpc() {
    
    if (is_stop_) {
        zcm_ipc.stop();
        return;
    }
    if (!zcm_ipc.good()) {
        cout << "zcm_ipc is not good!" << endl;
        return;
    }
    
    zcm_ipc.subscribe("MsgViewTrajectorySignal", &Handler::HandleMsgViewTrajectory,
                        &inner_handler);
    zcm_ipc.subscribe("MsgViewParkinglotListSignal",&Handler::HandleMsgViewParkinglotLists,
                        &inner_handler);
    zcm_ipc.subscribe("MsgViewObstacleListSignal",&Handler::HandleMsgViewObstacleList,
                        &inner_handler);

    zcm_ipc.subscribe("MsgViewRefLineSignal",&Handler::HandleMsgViewRefline,
                        &inner_handler);
    zcm_ipc.subscribe("MsgViewRefLineListSignal",&Handler::HandleMsgViewReflineList,
                        &inner_handler);
    zcm_ipc.subscribe("MsgViewStartSpotSignal",&Handler::HandleMsgViewStartSpot,
                        &inner_handler);
    
    zcm_ipc.subscribe("MsgFusionMapSignal",&Handler::HandleMsgFusionMap,
                        &inner_handler);

    // zcm_ipc.run();
    zcm_ipc.start();
    if (is_stop_) {
        zcm_ipc.stop();
    }
}

// pass zcm messages to view_startspot
bool ZcmMessageManager::GetViewStartSpot(MsgViewPoint &view_startspot) {
    inner_handler.startspot_mutex.lock();
    if (inner_handler.flag_received_startspot) {
        view_startspot = inner_handler.tmp_startspot;
        inner_handler.startspot_mutex.unlock();
        return true;
    }
    inner_handler.startspot_mutex.unlock();
    return false;
}

bool ZcmMessageManager::GetFusionMap(MsgFusionMap&fusion_map){
    fusion_map.map_cells.clear();
    inner_handler.fusion_map_mutex.lock();
    time_t current_time = getTimeStamp();
    if (inner_handler.flag_received_fusion_map)
    //  && current_time - inner_handler.update_time_fusion_map <
    //       TIMEOUT_US_FUSION_MAP) 
    {   
 
        fusion_map.map_row_num = inner_handler.tmp_fusion_map.map_row_num;
        fusion_map.map_column_num = inner_handler.tmp_fusion_map.map_column_num;
        fusion_map.map_resolution = inner_handler.tmp_fusion_map.map_resolution;
        fusion_map.car_center_row = inner_handler.tmp_fusion_map.car_center_row;
        fusion_map.car_center_column =  inner_handler.tmp_fusion_map.car_center_column;
        
        vector<uint8_t> temp(fusion_map.map_column_num);
        fusion_map.map_cells.resize(fusion_map.map_row_num,temp);
        for (size_t i = 0; i < fusion_map.map_row_num; i++)
        {
            for (size_t j = 0; j < fusion_map.map_column_num; j++)
            {
                uint8_t aa =  inner_handler.tmp_fusion_map.map_cells[i][j];
                fusion_map.map_cells[i].push_back(aa);
                // std::cout<<"aa:"<<(int)aa<<std::endl;
            }
            
        }
            //    std::cout<<"---------"<<inner_handler.tmp_fusion_map.map_cells.size()<<","<<fusion_map.map_cells.size()<<std::endl;
        
        inner_handler.fusion_map_mutex.unlock();
        return true;
    }
    inner_handler.fusion_map_mutex.unlock();
    return false;
}

bool ZcmMessageManager::GetViewRefline(MsgViewRefline &view_refline) {
    
    inner_handler.refline_mutex.lock();
    if (inner_handler.flag_received_refline) {
        view_refline = inner_handler.tmp_refline;
        inner_handler.refline_mutex.unlock();
        return true;
    }
    inner_handler.refline_mutex.unlock();
    return false;
}

// pass zcm messages to view_reflinelist
bool ZcmMessageManager::GetViewReflinelist(MsgViewReflineList &view_reflinelist) {
    
    inner_handler.reflinelist_mutex.lock();
    if (inner_handler.flag_received_reflinelist) {
        view_reflinelist = inner_handler.tmp_reflinelist;
        inner_handler.reflinelist_mutex.unlock();
        return true;
    }
    inner_handler.reflinelist_mutex.unlock();
    return false;
}

// pass zcm messages to view_trajectory
bool ZcmMessageManager::GetViewTrajectory(MsgViewTrajectory &view_trajectory) {
    inner_handler.trajectory_mutex.lock();
    if (inner_handler.flag_received_trajectory) {
        view_trajectory = inner_handler.tmp_trajectory;
        inner_handler.trajectory_mutex.unlock();
        return true;
    }
    inner_handler.trajectory_mutex.unlock();
    return false;
}

// pass zcm messages to view_parkinglotlist
bool ZcmMessageManager::GetViewParkinglotlist(MsgViewParkinglotList &view_parkinglotlist){
    inner_handler.parkinglotlist_mutex.lock();
    if (inner_handler.flag_received_parkinglotlist)
    {
        view_parkinglotlist = inner_handler.tmp_parkinglotlist;
        inner_handler.parkinglotlist_mutex.unlock();
        return true;
    }
    inner_handler.parkinglotlist_mutex.unlock();
    return false;
}

// pass zcm messages to view_obstaclelist
bool ZcmMessageManager::GetViewObstaclelist(MsgViewObstacleList &view_obstaclelist){
    inner_handler.obstaclelist_mutex.lock();
    if (inner_handler.flag_received_obstaclelist)
    {
        view_obstaclelist = inner_handler.tmp_obstaclelist;
        inner_handler.obstaclelist_mutex.unlock();
        return true;
    }
    inner_handler.obstaclelist_mutex.unlock();
    return false;
}

} // namespace planning
