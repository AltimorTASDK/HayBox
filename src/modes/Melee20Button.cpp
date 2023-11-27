#include "modes/Melee20Button.hpp"

//unified angles

Melee20Button::Melee20Button(socd::SocdType socd_type, Melee20ButtonOptions options) {
    socd_type = MELEE_SOCD;
    _socd_pair_count = 4;
    _socd_pairs = new socd::SocdPair[_socd_pair_count]{
        socd::SocdPair{&InputState::left,    &InputState::right,   socd_type},
        socd::SocdPair{ &InputState::down,   &InputState::up,      socd_type},
        socd::SocdPair{ &InputState::c_left, &InputState::c_right, socd_type},
        socd::SocdPair{ &InputState::c_down, &InputState::c_up,    socd_type},
    };

    _options = options;
    _horizontal_socd = false;
}

void Melee20Button::HandleSocd(InputState &inputs) {
    _horizontal_socd = inputs.left && inputs.right;
    InputMode::HandleSocd(inputs);
}

void Melee20Button::UpdateDigitalOutputs(InputState &inputs, OutputState &outputs) {
    outputs.a = inputs.a;
    outputs.b = inputs.b;
    outputs.x = inputs.x;
    outputs.y = inputs.y;
    outputs.buttonR = inputs.z;
    outputs.triggerLDigital = inputs.l;
    outputs.triggerRDigital = inputs.r;
    outputs.start = inputs.start;

    // Activate D-Pad layer by holding Mod X + Mod Y or Nunchuk C button.
    if ((inputs.mod_x && inputs.mod_y) || inputs.nunchuk_c) {
        outputs.dpadUp = inputs.c_up;
        outputs.dpadDown = inputs.c_down;
        outputs.dpadLeft = inputs.c_left;
        outputs.dpadRight = inputs.c_right;
    }

    if (inputs.select)
        outputs.dpadLeft = true;
    if (inputs.home)
        outputs.dpadRight = true;
}

void Melee20Button::UpdateAnalogOutputs(InputState &inputs, OutputState &outputs) {
    // Coordinate calculations to make modifier handling simpler.
    UpdateDirections(
        inputs.left,
        inputs.right,
        inputs.down,
        inputs.up,
        inputs.c_left,
        inputs.c_right,
        inputs.c_down,
        inputs.c_up,
        /*ANALOG_STICK_MIN*/ANALOG_STICK_NEUTRAL - 112,
        ANALOG_STICK_NEUTRAL,
        /*ANALOG_STICK_MAX*/ANALOG_STICK_NEUTRAL + 112,
        outputs
    );

    if (directions.diagonal) {
        // q1/2 = 7000 7000
        // actually 6875 7125 to account for randomness
        outputs.leftStickX = 128 + (directions.x * 56);
        outputs.leftStickY = 128 + (directions.y * 61);
        // L, R, LS, and MS + q3/4 = 7125 6875 (For vanilla shield drop. Gives 44.5
        // degree wavedash). Also used as default q3/4 diagonal if crouch walk option select is
        // enabled.
        // actually 7250 6750 to account for randomness
        if (directions.y == -1 && _options.crouch_walk_os) {
            outputs.leftStickX = 128 + (directions.x * 61);
            outputs.leftStickY = 128 + (directions.y * 56);
        }
    }

    if (inputs.mod_x) {
        // MX + Horizontal (even if shield is held) = 6625 = 53
        if (directions.horizontal) {
            outputs.leftStickX = 128 + (directions.x * 53);
        }
        // MX + Vertical (even if shield is held) = 5375 = 43
        // y=-0.5500 (44) is solo nana ice block, so we reduce this by one
        // MX + Vertical (even if shield is held) = 5250 = 42
        if (directions.vertical) {
            outputs.leftStickY = 128 + (directions.y * 42);
        }

        /* Up B angles */
        if (directions.diagonal) {
            if (inputs.l || inputs.r) {
                // 6625 3375 - 27.00 - 53 27 - modX + L/R
                // Wavedash travel time is handled by an angle restriction on output coords
                outputs.leftStickX = 128 + (directions.x * MX_SHIELD_TILT_X);
                outputs.leftStickY = 128 + (directions.y * MX_SHIELD_TILT_Y);
            } else if (!inputs.b) {
                // 7250 3125 - 23.32deg - 58 25 - modX
                // 7125 3750 - 27.76deg - 57 30 - modX + cDown
                // 6625 4125 - 31.91deg - 53 33 - modX + cLeft
                // 6625 4875 - 36.35deg - 53 39 - modX + cUp
                // 6125 5250 - 40.60deg - 49 42 - modX + cRight
                outputs.leftStickX = 128 + (directions.x * 58);
                outputs.leftStickY = 128 + (directions.y * 25);
                if (inputs.c_down) {
                    outputs.leftStickX = 128 + (directions.x * 57);
                    outputs.leftStickY = 128 + (directions.y * 30);
                }
                if (inputs.c_left) {
                    outputs.leftStickX = 128 + (directions.x * 53);
                    outputs.leftStickY = 128 + (directions.y * 33);
                }
                if (inputs.c_up) {
                    outputs.leftStickX = 128 + (directions.x * 53);
                    outputs.leftStickY = 128 + (directions.y * 39);
                }
                if (inputs.c_right) {
                    outputs.leftStickX = 128 + (directions.x * 49);
                    outputs.leftStickY = 128 + (directions.y * 42);
                }
            } else {
                /* Extended Up B Angles */
                // 9125 3875 - 23.01deg - 73 33 - modX + B
                // 8750 4625 - 27.86deg - 70 37 - modX + B + cDown
                // 8375 5250 - 32.08deg - 67 42 - modX + B + cLeft
                // 7125 5250 - 36.38deg - 57 42 - modX + B + cUp
                // 6125 5250 - 40.60deg - 49 42 - modX + B + cRight
                outputs.leftStickX = 128 + (directions.x * 73);
                outputs.leftStickY = 128 + (directions.y * 33);
                if (inputs.c_down) {
                    outputs.leftStickX = 128 + (directions.x * 70);
                    outputs.leftStickY = 128 + (directions.y * 37);
                }
                if (inputs.c_left) {
                    outputs.leftStickX = 128 + (directions.x * 67);
                    outputs.leftStickY = 128 + (directions.y * 42);
                }
                if (inputs.c_up) {
                    outputs.leftStickX = 128 + (directions.x * 57);
                    outputs.leftStickY = 128 + (directions.y * 42);
                }
                if (inputs.c_right) {
                    outputs.leftStickX = 128 + (directions.x * 49);
                    outputs.leftStickY = 128 + (directions.y * 42);
                }
            }
        }

        // Angled fsmash
        if (directions.cx != 0 && directions.y != 0) {
            // 8500 5250 = 68 42
            outputs.rightStickX = 128 + (directions.cx * 68);
            outputs.rightStickY = 128 + (directions.y * 42);
        }
    }

    if (inputs.mod_y) {
        // MY + Horizontal (even if shield is held) = 3375 = 27
        if (directions.horizontal) {
            outputs.leftStickX = 128 + (directions.x * 27);
        }
        // Turnaround neutral B nerf
        /* no more turnaround neutral b nerf
        if (inputs.b) {
            outputs.leftStickX = 128 + (directions.x * 80);
        }
        */
        // MY + Vertical (even if shield is held) = 7375 = 59
        if (directions.vertical) {
            outputs.leftStickY = 128 + (directions.y * 59);
        }

        /* Up B angles */
        if (directions.diagonal) {
            if (inputs.l || inputs.r) {
                // 3625 7125 - 26.97 - 29 57 - modX + L/R
                // Wavedash travel time is handled by an angle restriction on output coords
                outputs.leftStickX = 128 + (directions.x * MY_SHIELD_TILT_X);
                outputs.leftStickY = 128 + (directions.y * MY_SHIELD_TILT_Y);
            } else if (!inputs.b) {
                // 3250 7625 - 23.09deg - 26 61 - modY
                // 3625 7000 - 27.38deg - 29 56 - modY + cDown
                // 4375 7000 - 32.01deg - 35 56 - modY + cLeft
                // 5125 7000 - 36.21deg - 41 56 - modY + cUp
                // 5750 7125 - 38.90deg - 46 57 - modY + cRight
                outputs.leftStickX = 128 + (directions.x * 26);
                outputs.leftStickY = 128 + (directions.y * 61);
                if (inputs.c_down) {
                    outputs.leftStickX = 128 + (directions.x * 29);
                    outputs.leftStickY = 128 + (directions.y * 56);
                }
                if (inputs.c_left) {
                    outputs.leftStickX = 128 + (directions.x * 35);
                    outputs.leftStickY = 128 + (directions.y * 56);
                }
                if (inputs.c_up) {
                    outputs.leftStickX = 128 + (directions.x * 41);
                    outputs.leftStickY = 128 + (directions.y * 56);
                }
                if (inputs.c_right) {
                    outputs.leftStickX = 128 + (directions.x * 46);
                    outputs.leftStickY = 128 + (directions.y * 57);
                }
            } else {
                /* Extended Up B Angles */
                // 3875 9125 - 23.01deg - 31 73 - modY + B
                // 4625 8750 - 27.86deg - 37 70 - modY + B + cDown
                // 5250 8500 - 31.70deg - 42 68 - modY + B + cLeft
                // 5750 7875 - 36.14deg - 46 63 - modY + B + cUp
                // 5750 7125 - 38.90deg - 46 57 - modY + B + cRight
                outputs.leftStickX = 128 + (directions.x * 31);
                outputs.leftStickY = 128 + (directions.y * 73);
                if (inputs.c_down) {
                    outputs.leftStickX = 128 + (directions.x * 37);
                    outputs.leftStickY = 128 + (directions.y * 70);
                }
                if (inputs.c_left) {
                    outputs.leftStickX = 128 + (directions.x * 42);
                    outputs.leftStickY = 128 + (directions.y * 68);
                }
                if (inputs.c_up) {
                    outputs.leftStickX = 128 + (directions.x * 46);
                    outputs.leftStickY = 128 + (directions.y * 63);
                }
                if (inputs.c_right) {
                    outputs.leftStickX = 128 + (directions.x * 46);
                    outputs.leftStickY = 128 + (directions.y * 57);
                }
            }
        }
    }

    // C-stick ASDI Slideoff angle overrides any other C-stick modifiers (such as
    // angled fsmash).
    if (directions.cx != 0 && directions.cy != 0) {
        // 5250 8500 = 42 68
        outputs.rightStickX = 128 + (directions.cx * 42);
        outputs.rightStickY = 128 + (directions.cy * 68);
    }

    /*
    // Horizontal SOCD overrides X-axis modifiers (for ledgedash maximum jump
    // trajectory).
    if (_horizontal_socd && !directions.vertical) {
        outputs.leftStickX = 128 + (directions.x * 80);
    }
    */

    if (inputs.lightshield) {
        outputs.triggerRAnalog = 49;
    }
    if (inputs.midshield) {
        outputs.triggerRAnalog = 94;
    }

    if (outputs.triggerLDigital) {
        outputs.triggerLAnalog = 140;
    }
    if (outputs.triggerRDigital) {
        outputs.triggerRAnalog = 140;
    }

    // Shut off C-stick when using D-Pad layer.
    if ((inputs.mod_x && inputs.mod_y) || inputs.nunchuk_c) {
        outputs.rightStickX = 128;
        outputs.rightStickY = 128;
    }
}
