#ifndef _GAP_REMOTE_H_
#define _GAP_REMOTE_H_

#include <cstdint>

class GapRemote{
public:
    static GapRemote& GetInstance() {
        static GapRemote instance;
        return instance;
    }
    // 删除拷贝构造函数和赋值运算符
    GapRemote(const GapRemote&) = delete;
    GapRemote& operator=(const GapRemote&) = delete;
    int start_adv(uint8_t *adv_data, uint8_t len, uint32_t duration_ms=1000, bool force=true);
private:
    GapRemote();
    bool is_broadcast = false;
    uint8_t broadcast_count;
    uint8_t ble_addr_type;
    static void ble_sync_callback(void);
    static int ble_gap_event(struct ble_gap_event *event, void *arg);

};

#endif