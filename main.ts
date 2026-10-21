namespace nezhaV2_odometry {
    const MAX_POWER = 100;
    const MAX_CURVATURE = 1 / 150 // minimum radius: 150 mm
    const VELOCITY_FILTER_TIME_s = 0.08;
    let KFF_LEFT = 0.19;
    let KFF_RIGHT = 0.19;
    let KP_Speed = 0.1;
    let KP_Sync = 0.1;
    let KI_Speed = 0.3;
    let KI_Sync = 0.3;
    let Integral_Limit = MAX_POWER;

    let m_leftMainMotor = nezhaV2.MotorPostion.M3;
    let m_leftMotorDirection = 1;
    let m_leftSlaveMotor = nezhaV2.MotorPostion.M4;
    let m_rightMainMotor = nezhaV2.MotorPostion.M1;
    let m_rightMotorDirection = -1;
    let m_rightSlaveMotor = nezhaV2.MotorPostion.M2;
    let m_motionPerDeg_mm = 175;
    let m_wheelDistance_mm = 262;

    let m_velocityLeft_mm_s = 0;
    let m_velocityRight_mm_s = 0;
    let m_distanceLeft_mm = 0;
    let m_distanceRight_mm = 0;
    let m_encoderL = 0;
    let m_encoderR = 0;
    let m_lastEncUpdate = 0;
    let m_centerTravel_mm = 0;
    let m_odomX_mm = 0;
    let m_odomY_mm = 0;
    let m_odomHeading = 0;

    let m_lastUpdateControl = 0;
    let m_powerIntegralCommon = 0;
    let m_powerIntegralSync = 0;

    let m_debugTime = -1;
    let m_encoderInitialized = false;
    let m_controlInitialized = false;

    function setPower(left: number, right: number) {
        let leftPower = m_leftMotorDirection * left;
        let rightPower = m_rightMotorDirection * right;
        nezhaV2.start(m_leftMainMotor, leftPower);
        nezhaV2.start(m_rightMainMotor, rightPower);
        if(m_rightSlaveMotor > 0) nezhaV2.start(m_rightSlaveMotor, rightPower);
        if(m_leftSlaveMotor > 0) nezhaV2.start(m_leftSlaveMotor, leftPower);
    }

    function normalizeAngle(angle: number): number {
        return angle - 2 * Math.PI *
            Math.floor((angle + Math.PI) / (2 * Math.PI))
    }

    function clamp(x: number, lower: number, upper: number): number {
        return Math.max(lower, Math.min(upper, x));
    }

    function integrate(integral: number, error: number, gain: number, dt: number,
        requested: number = 0, limited: number = 0) {
        if (error * (requested - limited) <= 0) {
            integral += gain * error * dt;
        }
        return clamp(integral, -Integral_Limit, Integral_Limit);
    }

    //% blockId="odometry_driveController"
    //% block="Reguler til hastighed $speed_mm_s mm/s, efter kurve $curvature 1/mm"
    export function driveController(speed_mm_s: number, curvature: number): void {
        if (!m_encoderInitialized) return;

        let now = input.runningTime();
        let dt = 0;
        if (m_controlInitialized) {
            dt = (now - m_lastUpdateControl) / 1000;
        }

        m_lastUpdateControl = now;
        m_controlInitialized = true;

        if (dt < 0 || dt > 0.25) {
            m_powerIntegralCommon = 0;
            m_powerIntegralSync = 0;
            dt = 0;
        }

        let k = clamp(curvature, -MAX_CURVATURE, MAX_CURVATURE);
        let targetL = speed_mm_s * (1 - k * m_wheelDistance_mm / 2);
        let targetR = speed_mm_s * (1 + k * m_wheelDistance_mm / 2);
        let predictedLeft = KFF_LEFT * targetL;
        let predictedRight = KFF_RIGHT * targetR;

        let peakPower = Math.max(
            Math.abs(predictedLeft),
            Math.abs(predictedRight)
        );

        if (peakPower > MAX_POWER) {
            let scale = MAX_POWER / peakPower;
            targetL *= scale;
            targetR *= scale;
        }

        let targetCommon = (targetL + targetR) / 2;
        let targetDifferential = (targetL - targetR) / 2;

        let measuredCommon = (m_velocityLeft_mm_s + m_velocityRight_mm_s) / 2;
        let measuredDifferential = (m_velocityLeft_mm_s - m_velocityRight_mm_s) / 2;

        let errorCommon = targetCommon - measuredCommon;
        let errorSync = targetDifferential - measuredDifferential;

        let ffLeft = KFF_LEFT * targetL;
        let ffRight = KFF_RIGHT * targetR;

        let requestedCommon =
            (ffLeft + ffRight) / 2 +
            KP_Speed * errorCommon +
            m_powerIntegralCommon;

        let requestedSync =
            (ffLeft - ffRight) / 2 +
            KP_Sync * errorSync +
            m_powerIntegralSync;

        let appliedSync = clamp(requestedSync, -MAX_POWER, MAX_POWER);

        let commonHeadroom = MAX_POWER - Math.abs(appliedSync);

        let appliedCommon = clamp(requestedCommon, -commonHeadroom, commonHeadroom);

        m_powerIntegralCommon = integrate(m_powerIntegralCommon, errorCommon, KI_Speed, dt, requestedCommon, appliedCommon);
        m_powerIntegralSync = integrate(m_powerIntegralSync, errorSync, KI_Sync, dt, requestedSync, appliedSync);

        setPower(appliedCommon + appliedSync, appliedCommon - appliedSync);

        if(m_debugTime > 0 && (now - m_debugTime >= 100)) {
            m_debugTime = now;
            serial.writeLine("Speed: " + speed_mm_s + " Curvature: " + curvature);
            serial.writeLine("SpeedL: " + m_velocityLeft_mm_s + " SpeedR: " + m_velocityRight_mm_s);
            serial.writeLine("Common: " + appliedCommon + " Sync: " + appliedSync);
            serial.writeLine("X: " + m_odomX_mm + " Y: " + m_odomY_mm + " Angle: " + m_odomHeading);
        }
    }

    function updateOdometry(deltaLeft_mm: number, deltaRight_mm: number): void {
        let distance_mm = (deltaLeft_mm + deltaRight_mm) / 2;
        let deltaHeading = (deltaRight_mm - deltaLeft_mm) / m_wheelDistance_mm;

        m_centerTravel_mm += distance_mm;

        let halfTurn = deltaHeading / 2;
        let arcFactor = 1;

        if (Math.abs(halfTurn) < 0.0001) {
            arcFactor = 1 - halfTurn * halfTurn / 6;
        }
        else {
            arcFactor = Math.sin(halfTurn) / halfTurn;
        }

        let middleHeading = m_odomHeading + halfTurn;
        let chord = distance_mm * arcFactor;

        m_odomX_mm += chord * Math.cos(middleHeading);
        m_odomY_mm += chord * Math.sin(middleHeading);
        m_odomHeading = normalizeAngle(m_odomHeading + deltaHeading);
    }

    //% blockId="odometry_updateEncoder"
    //% block="Opdater odometri"
    export function updateEncoder(): void {
        if (m_leftMainMotor <= 0 ||
            m_rightMainMotor <= 0 ||
            m_motionPerDeg_mm <= 0 ||
            m_wheelDistance_mm <= 0) {
            return;
        }

        // Observe that it takes more than 4ms to read each encoder.
        let left = m_leftMotorDirection * nezhaV2.readRelAngle(m_leftMainMotor);
        let right = m_rightMotorDirection * nezhaV2.readRelAngle(m_rightMainMotor);
        let now = input.runningTime();
        if(!m_encoderInitialized) {
            m_encoderL = left;
            m_encoderR = right;
            m_lastEncUpdate = now;
            m_encoderInitialized = true;
            return;
        }

        let dt = (now - m_lastEncUpdate) / 1000;
        if (dt <= 0) return;
        
        let deltaLeft_mm = (left - m_encoderL) * m_motionPerDeg_mm;
        let deltaRight_mm = (right - m_encoderR) * m_motionPerDeg_mm;
        m_encoderL = left;
        m_encoderR = right;
        m_lastEncUpdate = now;

        m_distanceLeft_mm += deltaLeft_mm;
        m_distanceRight_mm += deltaRight_mm;

        let a = dt / (VELOCITY_FILTER_TIME_s + dt);
        m_velocityLeft_mm_s += a * (deltaLeft_mm / dt - m_velocityLeft_mm_s);
        m_velocityRight_mm_s += a * (deltaRight_mm / dt - m_velocityRight_mm_s);

        updateOdometry(deltaLeft_mm, deltaRight_mm);
    }

    //% blockId="odometry_stop"
    //% block="Stop motorer"
    export function stop(): void {
        if (m_leftMainMotor > 0)
            nezhaV2.stop(m_leftMainMotor);

        if (m_rightMainMotor > 0)
            nezhaV2.stop(m_rightMainMotor);

        if (m_leftSlaveMotor > 0)
            nezhaV2.stop(m_leftSlaveMotor);

        if (m_rightSlaveMotor > 0)
            nezhaV2.stop(m_rightSlaveMotor);

        m_powerIntegralCommon = 0;
        m_powerIntegralSync = 0;
        m_controlInitialized = false;
    }

    //% blockId="odometry_resetOdometry"
    //% block="Nulstil odometri"
    export function resetOdometry() : void {
        m_odomX_mm = 0;
        m_odomY_mm = 0;
        m_odomHeading = 0;

        m_centerTravel_mm = 0;
        m_distanceLeft_mm = 0;
        m_distanceRight_mm = 0;

        m_velocityLeft_mm_s = 0;
        m_velocityRight_mm_s = 0;

        m_encoderInitialized = false;
        m_controlInitialized = false;

        m_powerIntegralCommon = 0;
        m_powerIntegralSync = 0;
    }

    //% blockId="odometry_setLeftMotor"
    //% block="Indstil venstre motor $motor til fremadgående retning $direction | slave motor $slaveMotor"
    export function setLeftMotor(motor: nezhaV2.MotorPostion, direction: nezhaV2.MovementDirection, slaveMotor: nezhaV2.MotorPostion = 0): void {
        m_leftMainMotor = motor;
        m_leftMotorDirection = direction == nezhaV2.MovementDirection.CW ? 1 : -1;
        m_leftSlaveMotor = slaveMotor;
    }

    //% blockId="odometry_setRightMotor"
    //% block="Indstil højre motor $motor til fremadgående retning $direction | slave motor $slaveMotor"
    export function setRightMotor(motor: nezhaV2.MotorPostion, direction: nezhaV2.MovementDirection, slaveMotor: nezhaV2.MotorPostion = 0): void {
        m_rightMainMotor = motor;
        m_rightMotorDirection = direction == nezhaV2.MovementDirection.CW ? 1 : -1;
        m_rightSlaveMotor = slaveMotor;
    }

    //% blockId="odometry_setMotionPerTurn"
    //% block="Indstil hjulrotation per motor omdrejning $motionPerTurn_mm mm"
    export function setMotionPerTurn(motionPerTurn_mm: number): void {
        m_motionPerDeg_mm = motionPerTurn_mm / 360;
    }

    //% blockId="odometry_setWheelDistance"
    //% block="Indstil afstand mellem hjul $wheelDistance_mm mm"
    export function setWheelDistance(wheelDistance_mm: number): void {
        m_wheelDistance_mm = wheelDistance_mm;
    }

    //% blockId="odometry_setRegulationParameters"
    //% block="Indstil parametre $ffL $ffR $kpSpeed $kiSpeed $kpSync $kiSync"
    export function setRegulationParameters(ffL: number, ffR: number, kpSpeed: number, kiSpeed: number, kpSync: number, kiSync: number): void
    {
        KFF_LEFT = ffL;
        KFF_RIGHT = ffR;
        KP_Speed = kpSpeed;
        KI_Speed = kiSpeed;
        KP_Sync = kpSync;
        KI_Sync = kiSync;
    }

    //% blockId="odometry_getOdoX"
    //% block="X position mm"
    export function getOdoX(): number {
        return m_odomX_mm;
    }

    //% blockId="odometry_getOdoY"
    //% block="Y position mm"
    export function getOdoY(): number {
        return m_odomY_mm;
    }

    //% blockId="odometry_getHeading"
    //% block="Retning grader"
    export function getHeading(): number {
        return m_odomHeading * 180 / Math.PI;
    }

    //% blockId="odometry_getTrip"
    //% block="Trip mm"
    export function getTrip(): number {
        return m_centerTravel_mm;
    }

    //% blockId="odometry_getSpeed"
    //% block="Hastighed mm/s"
    export function getSpeed(): number {
        return (m_velocityLeft_mm_s + m_velocityRight_mm_s) / 2;
    }
}
