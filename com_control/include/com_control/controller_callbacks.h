#ifndef _COMCONTROL_CONTROLLER_CALLBACK_H
#define _COMCONTROL_CONTROLLER_CALLBACK_H

#include "com_control/message/response.h"

namespace crobot {

class ControllerCallbacks {
public:
    virtual void setSpeedCallback() = 0;

    virtual void getSpeedCallback(const GetSpeedResp &resp) = 0;
};

}

#endif  // _COMCONTROL_CONTROLLER_CALLBACK_H