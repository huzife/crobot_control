#ifndef _COMCONTROL_CONTROLLER_CALLBACK_H
#define _COMCONTROL_CONTROLLER_CALLBACK_H

#include "com_control/message/response.h"

namespace crobot {

class Controller_Callbacks {
public:
    virtual void set_speed_callback() = 0;

    virtual void get_speed_callback(const Get_Speed_Resp &resp) = 0;

    // virtual void getTempAndHumCallback(const GetTempAndHumResp &resp) = 0;
};

}

#endif  // _COMCONTROL_CONTROLLER_CALLBACK_H