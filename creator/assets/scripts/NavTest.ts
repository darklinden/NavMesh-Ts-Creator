import { _decorator, Component, input, Input, Vec3, EventMouse, resources, JsonAsset, Camera, geometry, PhysicsSystem } from "cc";
const { ccclass, property } = _decorator;

import NavMeshAgent from "./NavMeshAgent";
import NavMesh from "./NavMesh";

@ccclass
export default class NavTest extends Component {

    @property({ type: Camera })
    private camera: Camera = null;

    @property({ type: NavMeshAgent })
    private player: NavMeshAgent = null;

    private targetPos: Vec3;
    private playerNavMeshGroup: number;

    start(): void {
        resources.load('scene.navmesh', JsonAsset, (err, data) => {

            this.player.speed = 10;

            let zoneNodes = NavMesh.buildNodesByJson(data.json);
            NavMesh.setZoneData('game', zoneNodes);
            this.playerNavMeshGroup = NavMesh.getGroup('game', this.player.node.position);
        });

        input.on(Input.EventType.MOUSE_DOWN, this.onMouseDown, this);
    }


    private _ray: geometry.Ray = new geometry.Ray();
    private onMouseDown(event: EventMouse) {
        const location = event.getLocation();
        // 将屏幕坐标转化为射线

        this.camera.screenPointToRay(location.x, location.y, this._ray);
        if (PhysicsSystem.instance.raycast(this._ray)) {
            const raycastResults = PhysicsSystem.instance.raycastResults;
            for (let i = 0; i < raycastResults.length; i++) {
                const item = raycastResults[i];
                if (item.collider.node == this.node) {
                    this.onPosClicked(item.hitPoint);
                    break;
                }
            }
        } else {
            console.log('raycast does not hit the target node !');
        }
    }

    private onPosClicked(targetPos: Vec3) {
        this.targetPos = targetPos;
        let calculatedPath = NavMesh.findPath(this.player.node.position, this.targetPos, 'game', this.playerNavMeshGroup);

        if (calculatedPath && calculatedPath.length) {
            var debugPath = (calculatedPath);
            console.log("start", this.player.node.position.x, this.player.node.position.y, this.player.node.position.z);
            var p = [];
            for (var i = 0; i < debugPath.length; i++) {
                console.log(debugPath[i].x, debugPath[i].y, debugPath[i].z);
                p.push(new Vec3(debugPath[i].x, debugPath[i].y + .1, debugPath[i].z));
            }

            this.player.path = [this.player.node.position].concat(p);
            this.player.navEnabled = true;
            console.log("end", this.targetPos.x, this.targetPos.y, this.targetPos.z);
        }
        else {
            this.player.navEnabled = false;
        }
    }
}