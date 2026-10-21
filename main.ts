//% color=#2876A8 icon="\uf1b9" block="Robot"
namespace nezhaV2_odometry {
    const MAX_POWER = 100;
    const MAX_CURVATURE = 1 / 150 // minimum radius: 150 mm
    const VELOCITY_FILTER_TIME_s = 0.08;
    let KFF_LEFT = 0.19;
    let KFF_RIGHT = 0.19;
    let KP_Speed = 0.1;
    let KP_Sync = 0.08;
    let KI_Speed = 0.3;
    let KI_Sync = 0.3;
    let Integral_Limit = MAX_POWER;

    let m_leftMainMotor = nezhaV2.MotorPostion.M3;
    let m_leftMotorDirection = 1;
    let m_leftSlaveMotor = nezhaV2.MotorPostion.M4;
    let m_rightMainMotor = nezhaV2.MotorPostion.M1;
    let m_rightMotorDirection = -1;
    let m_rightSlaveMotor = nezhaV2.MotorPostion.M2;
    let m_motionPerDeg_mm = 175 / 360;
    let m_wheelDistance_mm = 451;

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
    let m_totalHeading = 0;

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
        if (m_rightSlaveMotor > 0) nezhaV2.start(m_rightSlaveMotor, rightPower);
        if (m_leftSlaveMotor > 0) nezhaV2.start(m_leftSlaveMotor, leftPower);
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
    //% block="Reguler hastighed $speed_mm_s mm/s, efter kurve $curvature 1/mm"
    //% advanced=true
    export function driveController(speed_mm_s: number, curvature: number): void {
        let k = clamp(curvature, -MAX_CURVATURE, MAX_CURVATURE);
        controlVelocity(speed_mm_s, speed_mm_s * k);
    }

    // Internal units: mm/s and radians/s. Supports rotation at zero speed.
    //% advanced=true
    //% blockId="odometry_controlVelocity"
    //% block="Reguler hastighed $speed_mm_s mm/s drejehastighed $turnRate_rad_s rad/s"
    export function controlVelocity(speed_mm_s: number, turnRate_rad_s: number): void {
        if (!m_encoderInitialized) return;
        let now = input.runningTime();
        if (now - m_lastEncUpdate > 250) {
            stop();
            return;
        }
        let dt = m_controlInitialized ? (now - m_lastUpdateControl) / 1000 : 0;
        m_lastUpdateControl = now;
        m_controlInitialized = true;
        if (dt < 0 || dt > 0.25) {
            m_powerIntegralCommon = 0;
            m_powerIntegralSync = 0;
            dt = 0;
        }
        let targetL = speed_mm_s - turnRate_rad_s * m_wheelDistance_mm / 2;
        let targetR = speed_mm_s + turnRate_rad_s * m_wheelDistance_mm / 2;
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

        if (m_debugTime > 0 && (now - m_debugTime >= 100)) {
            m_debugTime = now;
            serial.writeLine("Speed: " + speed_mm_s + " Turn rate: " + turnRate_rad_s);
            serial.writeLine("SpeedL: " + m_velocityLeft_mm_s + " SpeedR: " + m_velocityRight_mm_s);
            serial.writeLine("Common: " + appliedCommon + " Sync: " + appliedSync);
            serial.writeLine("X: " + m_odomX_mm + " Y: " + m_odomY_mm + " Angle: " + m_odomHeading);
        }
    }

    function updateOdometry(deltaLeft_mm: number, deltaRight_mm: number): void {
        let distance_mm = (deltaLeft_mm + deltaRight_mm) / 2;
        let deltaHeading = (deltaRight_mm - deltaLeft_mm) / m_wheelDistance_mm;

        m_centerTravel_mm += distance_mm;
        m_totalHeading += deltaHeading;

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
    //% advanced=true
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
        if (!m_encoderInitialized) {
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
        cancelMotion();
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
    //% advanced=true
    export function resetOdometry(): void {
        stop();
        m_totalHeading = 0;
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
    //% advanced=true
    export function setLeftMotor(motor: nezhaV2.MotorPostion, direction: nezhaV2.MovementDirection, slaveMotor: nezhaV2.MotorPostion = 0): void {
        m_leftMainMotor = motor;
        m_leftMotorDirection = direction == nezhaV2.MovementDirection.CW ? 1 : -1;
        m_leftSlaveMotor = slaveMotor;
    }

    //% blockId="odometry_setRightMotor"
    //% block="Indstil højre motor $motor til fremadgående retning $direction | slave motor $slaveMotor"
    //% advanced=true
    export function setRightMotor(motor: nezhaV2.MotorPostion, direction: nezhaV2.MovementDirection, slaveMotor: nezhaV2.MotorPostion = 0): void {
        m_rightMainMotor = motor;
        m_rightMotorDirection = direction == nezhaV2.MovementDirection.CW ? 1 : -1;
        m_rightSlaveMotor = slaveMotor;
    }

    //% blockId="odometry_setMotionPerTurn"
    //% block="Indstil hjulrotation per motor omdrejning $motionPerTurn_mm mm"
    //% advanced=true
    export function setMotionPerTurn(motionPerTurn_mm: number): void {
        if (motionPerTurn_mm > 0) m_motionPerDeg_mm = motionPerTurn_mm / 360;
    }

    //% blockId="odometry_setWheelDistance"
    //% block="Indstil afstand mellem hjul $wheelDistance_mm mm"
    //% advanced=true
    export function setWheelDistance(wheelDistance_mm: number): void {
        if (wheelDistance_mm > 0) m_wheelDistance_mm = wheelDistance_mm;
    }

    //% blockId="odometry_setRegulationParameters"
    //% block="Indstil parametre $ffL $ffR $kpSpeed $kiSpeed $kpSync $kiSync"
    //% advanced=true
    export function setRegulationParameters(ffL: number, ffR: number, kpSpeed: number, kiSpeed: number, kpSync: number, kiSync: number): void {
        KFF_LEFT = ffL;
        KFF_RIGHT = ffR;
        KP_Speed = kpSpeed;
        KI_Speed = kiSpeed;
        KP_Sync = kpSync;
        KI_Sync = kiSync;
        m_powerIntegralCommon = 0;
        m_powerIntegralSync = 0;
    }

    //% blockId="odometry_getOdoX"
    //% block="X position mm"
    //% advanced=true
    export function getOdoX(): number {
        return m_odomX_mm;
    }

    //% blockId="odometry_getOdoY"
    //% block="Y position mm"
    //% advanced=true
    export function getOdoY(): number {
        return m_odomY_mm;
    }

    //% blockId="odometry_getHeading"
    //% block="Retning grader"
    //% advanced=true
    export function getHeading(): number {
        return m_odomHeading * 180 / Math.PI;
    }

    //% blockId="odometry_getTrip"
    //% block="Trip mm"
    //% advanced=true
    export function getTrip(): number {
        return m_centerTravel_mm;
    }

    //% blockId="odometry_getSpeed"
    //% block="Hastighed mm/s"
    //% advanced=true
    export function getSpeed(): number {
        return (m_velocityLeft_mm_s + m_velocityRight_mm_s) / 2;
    }


    export enum TravelDirection {
        //% block="fremad"
        Forward = 1,
        //% block="baglæns"
        Backward = -1
    }
    export enum TurnDirection {
        //% block="venstre"
        Left = 1,
        //% block="højre"
        Right = -1
    }
    const PERIOD_MS = 40;
    const HEADING_GAIN = 0.015;
    const LATERAL_GAIN = 0.0001;
    let mode = 0; // 0 stopped, 1 path, 2 rotation, 3 external provider
    let commandId = 0;
    let loopStarted = false;
    let finiteMove = false;
    let deadline = 0;
    let startTrip = 0;
    let startAngle = 0;
    let startX = 0;
    let startY = 0;
    let startHeading = 0;
    let pathCurvature = 0;
    let commandedSpeed = 0;
    let turnSign = 1;
    let goal = 0;
    let settledAt = -1;
    let lastSuccess = true;
    let provider: () => boolean = null;
    let externalSpeed = 0;
    let externalTurn = 0;

    // Called by stop() in main.ts. Invalidates every old waiting command.
    function cancelMotion(): void {
        mode = 0;
        commandId++;
        provider = null;
        lastSuccess = false;
    }

    function beginMotion(): void {
        stop();
        updateEncoder();
        startTrip = m_centerTravel_mm;
        startAngle = m_totalHeading;
        startX = m_odomX_mm;
        startY = m_odomY_mm;
        startHeading = m_odomHeading;
        settledAt = -1;
        lastSuccess = false;
        if (!loopStarted) {
            loopStarted = true;
            control.inBackground(function () {
                while (true) {
                    let started = input.runningTime();
                    updateEncoder();
                    if (mode != 0) stepMotion();
                    basic.pause(Math.max(1, PERIOD_MS - (input.runningTime() - started)));
                }
            });
        }
    }

    function finishMotion(success: boolean): void {
        stop();
        lastSuccess = success;
    }

    function settled(condition: boolean): boolean {
        if (!condition) {
            settledAt = -1;
            return false;
        }
        if (settledAt < 0) settledAt = input.runningTime();
        return input.runningTime() - settledAt >= 200;
    }

    function stepMotion(): void {
        if (!m_encoderInitialized) {
            finishMotion(false);
            return;
        }
        if (finiteMove && input.runningTime() > deadline) {
            finishMotion(false);
            return;
        }
        if (mode == 3) {
            if (!provider || !provider()) {
                finishMotion(false);
                return;
            }
            controlVelocity(externalSpeed, externalTurn);
            return;
        }
        if (mode == 2) {
            let remaining = goal - turnSign * (m_totalHeading - startAngle);
            if (settled(Math.abs(remaining) < Math.PI / 180 &&
                Math.abs(m_velocityLeft_mm_s) < 5 && Math.abs(m_velocityRight_mm_s) < 5)) {
                finishMotion(true);
                return;
            }
            // Allow a small reverse correction after overshoot.
            controlVelocity(0, turnSign * clamp(3 * remaining, -commandedSpeed, commandedSpeed));
            return;
        }
        let speed = commandedSpeed;
        if (finiteMove) {
            let progress = (commandedSpeed >= 0 ? 1 : -1) * (m_centerTravel_mm - startTrip);
            let remaining = goal - progress;
            if (settled(remaining <= 2 && Math.abs(m_velocityLeft_mm_s) < 5 &&
                Math.abs(m_velocityRight_mm_s) < 5)) {
                finishMotion(true);
                return;
            }
            speed = (commandedSpeed >= 0 ? 1 : -1) *
                Math.min(Math.abs(commandedSpeed), Math.max(0, 3 * remaining));
        }
        let refHeading = startHeading;
        let refX = startX;
        let refY = startY;
        if (Math.abs(pathCurvature) > 0.000001) {
            let cx = startX - Math.sin(startHeading) / pathCurvature;
            let cy = startY + Math.cos(startHeading) / pathCurvature;
            let radial = Math.atan2(m_odomY_mm - cy, m_odomX_mm - cx);
            let radius = Math.abs(1 / pathCurvature);
            refX = cx + radius * Math.cos(radial);
            refY = cy + radius * Math.sin(radial);
            refHeading = radial + (pathCurvature > 0 ? 1 : -1) * Math.PI / 2;
        }
        let lateral = -Math.sin(refHeading) * (m_odomX_mm - refX) +
            Math.cos(refHeading) * (m_odomY_mm - refY);
        let headingError = normalizeAngle(m_odomHeading - refHeading);
        // Signed lateral term and unsigned damping support forward AND reverse.
        let omega = speed * pathCurvature - Math.abs(speed) * HEADING_GAIN * headingError -
            speed * LATERAL_GAIN * lateral;
        let maxOmega = Math.abs(speed) / 100;
        controlVelocity(speed, clamp(omega, -maxOmega, maxOmega));
    }

    function startPath(direction: TravelDirection, speed: number, curvature: number,
        distance: number, limited: boolean): void {
        beginMotion();
        if (speed <= 0 || (limited && distance <= 0)) return;
        commandedSpeed = (direction == TravelDirection.Forward ? 1 : -1) * speed;
        pathCurvature = curvature;
        finiteMove = limited;
        goal = distance;
        deadline = input.runningTime() + 5000 + 4000 * distance / speed;
        mode = 1;
    }

    function waitForCommand(id: number): void {
        while (mode != 0 && commandId == id) basic.pause(10);
    }

    //% blockId="robot_startStraight"
    //% block="Start kørsel $direction med $speed mm/s"
    //% speed.defl=150 speed.min=1
    export function startStraight(direction: TravelDirection, speed: number): void {
        startPath(direction, speed, 0, 0, false);
    }

    //% blockId="robot_driveDistance"
    //% block="Kør $direction $distance mm med $speed mm/s"
    //% distance.defl=300 distance.min=0 speed.defl=150 speed.min=1
    export function driveDistance(direction: TravelDirection, distance: number, speed: number): void {
        startPath(direction, speed, 0, distance, true);
        waitForCommand(commandId);
    }

    // Left/right refers to the direction the robot's FRONT turns, also in reverse.
    // Radius is measured to the centre of the axle. Minimum 150 mm.
    //% blockId="robot_startCircle"
    //% block="Start cirkel $direction mod $turn radius $radius mm med $speed mm/s"
    //% radius.defl=300 radius.min=150 speed.defl=150 speed.min=1
    export function startCircle(direction: TravelDirection, turn: TurnDirection,
        radius: number, speed: number): void {
        if (radius < 150) { stop(); return; }
        let k = (turn == TurnDirection.Left ? 1 : -1) *
            (direction == TravelDirection.Forward ? 1 : -1) / radius;
        startPath(direction, speed, k, 0, false);
    }

    // Arc completion is based on centre travel, not final heading.
    //% blockId="robot_driveArc"
    //% block="Kør $direction i bue mod $turn radius $radius mm gennem $degrees ° med $speed mm/s"
    //% radius.defl=300 radius.min=150 degrees.defl=90 degrees.min=0 speed.defl=150 speed.min=1
    export function driveArc(direction: TravelDirection, turn: TurnDirection,
        radius: number, degrees: number, speed: number): void {
        if (radius < 150 || degrees < 0) { stop(); return; }
        let k = (turn == TurnDirection.Left ? 1 : -1) *
            (direction == TravelDirection.Forward ? 1 : -1) / radius;
        startPath(direction, speed, k, radius * degrees * Math.PI / 180, true);
        waitForCommand(commandId);
    }

    //% blockId="robot_rotate"
    //% block="Drej på stedet $turn $degrees °"
    //% degrees.defl=90 degrees.min=0
    export function rotate(turn: TurnDirection, degrees: number): void {
        beginMotion();
        let degreesPerSecond = 20;
        if (degrees <= 0 || degreesPerSecond <= 0) return;
        goal = degrees * Math.PI / 180;
        commandedSpeed = degreesPerSecond * Math.PI / 180;
        turnSign = turn == TurnDirection.Left ? 1 : -1;
        finiteMove = true;
        deadline = input.runningTime() + 5000 + 4000 * degrees / degreesPerSecond;
        mode = 2;
        waitForCommand(commandId);
    }

    // Adapter API: callback runs in the SAME loop as motor I/O.
    // Return false to stop. Do not pause or issue movement commands in callback.
    export function startExternalController(update: () => boolean): void {
        beginMotion();
        provider = update;
        externalSpeed = 0;
        externalTurn = 0;
        finiteMove = false;
        mode = 3;
    }

    export function setExternalVelocity(speed: number, turnRate: number): void {
        externalSpeed = speed;
        externalTurn = turnRate;
    }

    //% blockId="robot_busy"
    //% block="robotten kører?"
    export function isMoving(): boolean { return mode != 0; }

    //% advanced=true blockId="robot_success" block="sidste bevægelse gennemført?"
    export function motionSucceeded(): boolean { return lastSuccess; }
}
