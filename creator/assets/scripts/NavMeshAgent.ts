import { _decorator, Component, Node, Vec3 } from "cc";
const { ccclass, property } = _decorator;

@ccclass
export default class NavMeshAgent extends Component {

    @property({ type: Node })
    private posEnd: Node = null;

    navMeshGroup = null;
    public navEnabled = false;
    updateRotation = false;
    _pathPending = false;
    // 路线进行中
    _path: Vec3[] = null;
    _pathp = 0;
    _pathlen = 0;
    _remainingDistance = 1;
    destination = null;
    speed = 1;
    steeringTarget = new Vec3();
    _velocity = new Vec3();
    out = new Vec3();

    start(): void {
        this.navMeshGroup = null;
        this.navEnabled = false;
        this.updateRotation = false;
        this._pathPending = false;
        // 路线进行中
        this._path = null;
        this._pathp = 0;
        this._pathlen = 0;
        this._remainingDistance = 1;
        this.destination = null;
        this.speed = 1;
        this.steeringTarget = new Vec3();
        this._velocity = new Vec3();
        this.out = new Vec3();
    }

    update(dt: number): void {
        if (this.navEnabled) {
            var now = this.node.position;
            if (this._path) {
                var v = new Vec3;
                var tp = null;
                for (var i = this._pathp; i < this._path.length - 1; i++) {
                    var p0 = this._path[i];
                    var p1 = this._path[i + 1];
                    this._pathlen = this._pathlen + this.speed / 60;
                    var tlen = Vec3.distance(p0, p1);
                    if (this._pathlen > tlen) {
                        this._pathlen -= tlen;
                        this._pathp++;
                    } else {
                        tp = p0.clone();
                        this.steeringTarget = p1.clone();
                        Vec3.subtract(v, p1, p0);
                        Vec3.normalize(v, v);
                        Vec3.multiplyScalar(v, v, this._pathlen);
                        Vec3.add(tp, p0, v);
                        break;
                    }
                }
                if (tp == null) {
                    this._pathPending = false;
                    tp = this._path[this._path.length - 1];
                    this.steeringTarget = this._path[this._path.length - 1].clone();
                }
                this.node.position = tp;
            } else {
                if (this.navMeshGroup == null) {
                    this.node.position = this.steeringTarget;
                }
            }
        }
    }

    get remainingDistance() {
        if (this.destination) {
            return Vec3.distance(this.destination, this.node.position);
        }
        return this._remainingDistance;
    }

    set remainingDistance(value) {
        this._remainingDistance = value;
    }

    get velocity() {
        return this._velocity;
    }

    set velocity(value) {
        this._velocity = value;
        this.destination = null;
    }

    get path() {
        return this._path;
    }

    set path(value) {
        this._path = value;
        if (value) {
            this._pathPending = true;
            this.node.position = this._path[0].clone();
            this.posEnd.position = this._path[this._path.length - 1].clone();
        } else {
            this._pathPending = false;
        }
        this._pathp = 0;
        this._pathlen = 0;
    }
}