import { IJoystickUpdateEvent } from "../pages/App";

export type Vector3 = {
    x: number;
    y: number;
    z: number;
}

export type TwistMessage = {
    linear: Vector3;
    angular: Vector3;
};


export const convertJoystickDataToTwistMessage = (joystickData: IJoystickUpdateEvent): TwistMessage => {
    return {
        linear: {
            x: joystickData.x ?? 0,
            y: joystickData.y ?? 0,
            z: 0,
        },
        angular: {
            x: 0,
            y: 0,
            z: 0,
        },
    };
}