namespace nezhaV2_odometry {
    export enum MotionDirection {
        Forward = 1,
        Reverse = 2
    }

    export enum MotionCompleteBehaviour {
        Stop = 1,
        KeepGoing = 2
    }

    let VelocityFilterTime_s = 0.05;

    let m_leftMainMotor = 0;
    let m_leftMotorDirection = 0;
    let m_leftSlaveMotor = 0;
    let m_rightMainMotor = 0;
    let m_rightMotorDirection = 0;
    let m_rightSlaveMotor = 0;
    let m_motionPerDeg_mm = 0;
    let m_wheelDistance_mm = 0;

    let m_velocityLeft = 0;
    let m_velocityRight = 0;
    let m_distanceLeft_mm = 0;
    let m_distanceRight_mm = 0;
    let m_encoderL = 0;
    let m_encoderR = 0;
    let m_lastUpdateL = 0;
    let m_lastUpdateR = 0;

    function controller(): void {
        
    }

    function updateOdometry(deltaLeft_mm: number, deltaRight_mm: number): void {
        // ToDo
    }

    function updateEncoder(): void {
        // Observe that it takes more than 4ms to read each encoder.
        // Wrap-around ??
        let left = (m_leftMotorDirection == nezhaV2.MovementDirection.CW ? 1 : -1) *
            nezhaV2.readRelAngle(m_leftMainMotor);
        let now = input.runningTimeMicros();
        let deltaLeft_mm = (left - m_encoderL) * m_motionPerDeg_mm;
        m_encoderL = left;
        m_distanceLeft_mm += deltaLeft_mm;
        let dt = (now - m_lastUpdateL) / 1000000;
        m_lastUpdateL = now;
        let a = dt / (VelocityFilterTime_s + dt);
        m_velocityLeft += a * (deltaLeft_mm / dt - m_velocityLeft);

        let right = (m_rightMotorDirection == nezhaV2.MovementDirection.CW ? 1 : -1) *
            nezhaV2.readRelAngle(m_rightMainMotor);
        now = input.runningTimeMicros();
        let deltaRight_mm = (left - m_encoderR) * m_motionPerDeg_mm;
        m_encoderR = right;
        m_distanceRight_mm = deltaRight_mm;
        dt = (now - m_lastUpdateL) / 1000000;
        m_lastUpdateR = now;
        a = dt / (VelocityFilterTime_s + dt);
        m_velocityRight += a * (deltaRight_mm / dt - m_velocityRight);

        updateOdometry(deltaLeft_mm, deltaRight_mm);
    }

    export function setLeftMotor(motor: nezhaV2.MotorPostion, direction: nezhaV2.MovementDirection, slaveMotor: nezhaV2.MotorPostion = 0): void {
        m_leftMainMotor = motor;
        m_leftMotorDirection = direction;
        m_leftSlaveMotor = slaveMotor;
    }

    export function setRightMotor(motor: nezhaV2.MotorPostion, direction: nezhaV2.MovementDirection, slaveMotor: nezhaV2.MotorPostion = 0): void {
        m_rightMainMotor = motor;
        m_rightMotorDirection = direction;
        m_rightSlaveMotor = slaveMotor;
    }

    export function setMotionPerTurn(motionPerTurn_mm: number): void {
        m_motionPerDeg_mm = motionPerTurn_mm / 360;
    }

    export function setWheelDistance(wheelDistance_mm: number): void {
        m_wheelDistance_mm = wheelDistance_mm;
    }

    export function driveStraight(direction: MotionDirection, speed_mm_s: number,
        distance_mm: number = Infinity,
        completeBehaviour: MotionCompleteBehaviour = MotionCompleteBehaviour.Stop): void
    {
        
    }
}
