import { _decorator, Component, input, Input, EventKeyboard, KeyCode, Vec3, Mat4, __private, EventMouse, Vec2, Quat, Toggle } from "cc";
const { ccclass } = _decorator;

const RIGHT = Object.freeze(new Vec3(10, 0, 0));
const LEFT = Object.freeze(new Vec3(-10, 0, 0));
const FORWARD = Object.freeze(new Vec3(0, 0, -10)); // we use -z for view-dir
const BACK = Object.freeze(new Vec3(0, 0, 10));
const UP = Object.freeze(new Vec3(0, 10, 0));
const DOWN = Object.freeze(new Vec3(0, -10, 0));

const ROTATION_SPEED = 0.01;

// --------- func from laya -----------
function quatArcTanAngle(x: number, y: number): number {
    if (x == 0) {
        if (y == 1)
            return Math.PI / 2;
        return -Math.PI / 2;
    }
    if (x > 0)
        return Math.atan(y / x);
    if (x < 0) {
        if (y > 0)
            return Math.atan(y / x) + Math.PI;
        return Math.atan(y / x) - Math.PI;
    }
    return 0;
}

function quatAngleTo(from: Vec3, location: Vec3, angleOut: Vec3) {
    const tmp = new Vec3();
    Vec3.subtract(tmp, location, from);
    tmp.normalize();
    angleOut.x = Math.asin(tmp.y);
    angleOut.y = quatArcTanAngle(-tmp.z, -tmp.x);
}

function vec3TransformQuat(source: Vec3, rotation: Quat, out: Vec3) {
    var x = source.x, y = source.y, z = source.z, qx = rotation.x, qy = rotation.y, qz = rotation.z, qw = rotation.w, ix = qw * x + qy * z - qz * y, iy = qw * y + qz * x - qx * z, iz = qw * z + qx * y - qy * x, iw = -qx * x - qy * y - qz * z;
    out.x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
    out.y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
    out.z = iz * qw + iw * -qz + ix * -qy - iy * -qx;
}

// from laya
function getYawPitchRoll(in_: Quat, out: Quat) {
    const forward = new Vec3();
    const upe = new Vec3();
    const angle = new Vec3();
    Vec3.transformQuat(forward, Vec3.FORWARD, in_);
    Vec3.transformQuat(upe, Vec3.UP, in_);
    quatAngleTo(Vec3.ZERO, forward, angle);
    if (angle.x == Math.PI / 2) {
        angle.y = quatArcTanAngle(upe.z, upe.x);
        angle.z = 0;
    }
    else if (angle.x == -Math.PI / 2) {
        angle.y = quatArcTanAngle(-upe.z, -upe.x);
        angle.z = 0;
    }
    else {
        const mat0 = new Mat4();
        const mat1 = new Mat4();
        Mat4.rotateY(mat0, mat0, -angle.y);
        Mat4.rotateX(mat1, mat1, -angle.x);
        Vec3.transformMat4(upe, upe, mat0);
        Vec3.transformMat4(upe, upe, mat1);
        angle.z = quatArcTanAngle(upe.y, -upe.x);
    }
    if (angle.y <= -Math.PI)
        angle.y = Math.PI;
    if (angle.z <= -Math.PI)
        angle.z = Math.PI;
    if (angle.y >= Math.PI && angle.z >= Math.PI) {
        angle.y = 0;
        angle.z = 0;
        angle.x = Math.PI - angle.x;
    }

    out.x = angle.y;
    out.y = angle.x;
    out.z = angle.z;
}

function createFromYawPitchRoll(yaw: number, pitch: number, roll: number, out: Quat): void {
    var halfRoll = roll * 0.5;
    var halfPitch = pitch * 0.5;
    var halfYaw = yaw * 0.5;
    var sinRoll = Math.sin(halfRoll);
    var cosRoll = Math.cos(halfRoll);
    var sinPitch = Math.sin(halfPitch);
    var cosPitch = Math.cos(halfPitch);
    var sinYaw = Math.sin(halfYaw);
    var cosYaw = Math.cos(halfYaw);
    out.x = (cosYaw * sinPitch * cosRoll) + (sinYaw * cosPitch * sinRoll);
    out.y = (sinYaw * cosPitch * cosRoll) - (cosYaw * sinPitch * sinRoll);
    out.z = (cosYaw * cosPitch * sinRoll) - (sinYaw * sinPitch * cosRoll);
    out.w = (cosYaw * cosPitch * cosRoll) + (sinYaw * sinPitch * sinRoll);
}

// --------- func from laya -----------

@ccclass
export class CameraMoveDebug extends Component {

    private _enable: boolean = true;
    public SetActiveByToggle(t: Toggle): void {
        this._enable = t.isChecked;
    }

    start() {
        input.on(Input.EventType.KEY_DOWN, this.onKeyDown, this);
        input.on(Input.EventType.KEY_UP, this.onKeyUp, this);
        input.on(Input.EventType.MOUSE_DOWN, this.onMouseDown, this);
        input.on(Input.EventType.MOUSE_UP, this.onMouseUp, this);
        input.on(Input.EventType.MOUSE_MOVE, this.onMouseMove, this);
    }

    private _speed: Vec3 = new Vec3();
    private onKeyDown(event: EventKeyboard) {
        console.log('MoveDebug.onKeyDown', event.keyCode);
        if (event.keyCode == KeyCode.KEY_W) { this._speed.add(FORWARD); }
        if (event.keyCode == KeyCode.KEY_S) { this._speed.add(BACK); }
        if (event.keyCode == KeyCode.KEY_A) { this._speed.add(LEFT); }
        if (event.keyCode == KeyCode.KEY_D) { this._speed.add(RIGHT); }
        if (event.keyCode == KeyCode.SPACE) { this._speed.add(UP); }
        if (event.keyCode == KeyCode.SHIFT_LEFT || event.keyCode == KeyCode.SHIFT_RIGHT) { this._speed.add(DOWN); }
    }

    private onKeyUp(event: EventKeyboard) {
        console.log('MoveDebug.onKeyUp', event.keyCode);
        if (event.keyCode == KeyCode.KEY_W) { this._speed.subtract(FORWARD); }
        if (event.keyCode == KeyCode.KEY_S) { this._speed.subtract(BACK); }
        if (event.keyCode == KeyCode.KEY_A) { this._speed.subtract(LEFT); }
        if (event.keyCode == KeyCode.KEY_D) { this._speed.subtract(RIGHT); }
        if (event.keyCode == KeyCode.SPACE) { this._speed.subtract(UP); }
        if (event.keyCode == KeyCode.SHIFT_LEFT || event.keyCode == KeyCode.SHIFT_RIGHT) { this._speed.subtract(DOWN); }
    }

    private _isMouseDown: boolean = false;
    private _lastMouse: Vec2 = null;
    private _yawPitchRoll: Quat = new Quat();
    private _tempRotationZ: Quat = new Quat();

    private onMouseDown(event: EventMouse) {
        getYawPitchRoll(this.node.rotation, this._yawPitchRoll);
        this._lastMouse = event.getLocation().clone();
        this._isMouseDown = true;
    }

    private onMouseUp(event: EventMouse) {
        this._isMouseDown = false;
    }

    private onMouseMove(event: EventMouse) {
        if (!this._enable) return;

        if (this._isMouseDown) {
            const current = event.getLocation().clone();
            if (this._lastMouse) {

                var offsetX = current.x - this._lastMouse.x;
                var offsetY = current.y - this._lastMouse.y;

                var yprElem = this._yawPitchRoll;
                yprElem.x -= offsetX * ROTATION_SPEED;
                yprElem.y += offsetY * ROTATION_SPEED;

                if (Math.abs(yprElem.y) < 1.50) {
                    createFromYawPitchRoll(yprElem.x, yprElem.y, yprElem.z, this._tempRotationZ);
                    this.node.rotation = this._tempRotationZ;
                }
            }
            this._lastMouse = current;
        }
    }

    update(dt: number) {
        if (!this._enable) return;

        if (this._speed && !this._speed.equals(Vec3.ZERO)) {
            this.node.translate(Vec3.multiplyScalar(new Vec3(), this._speed, dt));
        }
    }
}
