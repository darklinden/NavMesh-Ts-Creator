import { IVec3Like, Vec3 } from "cc";
import Heap from "./Heap";

// from https://github.com/nickjanssen/PatrolJS

export namespace NavMesh {

    // 从 obj 读取的 面 数据
    export interface IFace {
        a: number;
        b: number;
        c: number;
        centroid?: IVec3Like;
        normal?: IVec3Like;
    }

    function face(a: number, b: number, c: number): IFace {
        return { a: a, b: b, c: c } as IFace
    }

    // 用来计算的三角形，从 面 数据转换而来
    export interface ITriangle {
        id: number;
        vertexIds: [number, number, number];
        centroid: IVec3Like;
        normal?: IVec3Like;
        neighbours?: ITriangle[];
        group?: number;

        g?: number;
        f?: number;
        h?: number;
        cost?: number;
        visited?: boolean;
        closed?: boolean;
        parent?: ITriangle;
    }

    function triangleFromFace(id: number, face: IFace): ITriangle {
        return {
            id: id,
            vertexIds: [face.a, face.b, face.c],
            centroid: face.centroid,
            normal: face.normal,
        } as ITriangle;
    }

    function resetTriangle(n: ITriangle) {
        n.f = 0;
        n.g = 0;
        n.h = 0;
        n.cost = 1.0;
        n.visited = false;
        n.closed = false;
        n.parent = null;
    }

    function cleanTriangle(n: ITriangle) {
        delete n.f;
        delete n.g;
        delete n.h;
        delete n.cost;
        delete n.visited;
        delete n.closed;
        delete n.parent;
    }

    export interface ITriangleInfo {
        triangles: ITriangle[];
        vertices: IVec3Like[];
    }

    export interface IWavefront {
        faces: IFace[];
        vertices: IVec3Like[];
    }

    // return changed count 
    function mergeVertices(obj: IWavefront): number {
        const verticesMap = {};
        const unique: IVec3Like[] = [];
        const changes: number[] = [];
        const precisionPoints = 4;
        const precision = Math.pow(10, precisionPoints);

        let il = obj.vertices.length;
        for (let i = 0; i < il; i++) {
            const v = obj.vertices[i];
            const key = Math.round(v.x * precision) + '_' + Math.round(v.y * precision) + '_' + Math.round(v.z * precision);
            if (verticesMap[key] == null) {
                verticesMap[key] = i;
                unique.push(v);
                changes[i] = unique.length - 1;
            } else {
                changes[i] = changes[verticesMap[key]];
            }
        }

        il = obj.faces.length;
        let faceIndicesToRemove = [];
        for (let i = 0; i < il; i++) {
            const face = obj.faces[i];
            face.a = changes[face.a];
            face.b = changes[face.b];
            face.c = changes[face.c];
            const indices = [face.a, face.b, face.c];
            let dupIndex = -1;
            for (let n = 0; n < 3; n++) {
                if (indices[n] == indices[(n + 1) % 3]) {
                    dupIndex = n;
                    faceIndicesToRemove.push(i);
                    break;
                }
            }
        }

        for (let i = faceIndicesToRemove.length - 1; i >= 0; i--) {
            let idx = faceIndicesToRemove[i];
            obj.faces.splice(idx, 1);
        }

        let diff = obj.vertices.length - unique.length;
        obj.vertices = unique;
        return diff;
    }

    function computeCentroids(obj: IWavefront) {
        for (let f = 0, fl = obj.faces.length; f < fl; f++) {
            const face = obj.faces[f];
            face.centroid = { x: 0, y: 0, z: 0 } as IVec3Like;
            Vec3.add(face.centroid, face.centroid, obj.vertices[face.a]);
            Vec3.add(face.centroid, face.centroid, obj.vertices[face.b]);
            Vec3.add(face.centroid, face.centroid, obj.vertices[face.c]);
            Vec3.multiplyScalar(face.centroid, face.centroid, 1 / 3);
        }
    }

    function vertexDuplicate(a: [number, number, number], b: [number, number, number]) {
        const ret: number[] = [];
        for (let i = 0; i < a.length; i++)
            if (b.indexOf(a[i]) != -1) { ret.push(a[i]) }

        return ret;
    }

    function buildTriangleNeighbours(polygon: ITriangle, navigationMesh: ITriangleInfo): void {

        polygon.neighbours = [];

        // All other nodes that contain at least two of our vertices are our neighbours
        for (let i = 0, len = navigationMesh.triangles.length; i < len; i++) {
            if (polygon === navigationMesh.triangles[i]) continue;

            // Don't check polygons that are too far, since the intersection tests take a long time
            if (Vec3.squaredDistance(polygon.centroid, navigationMesh.triangles[i].centroid) > 100 * 100) continue;

            let matches = vertexDuplicate(polygon.vertexIds, navigationMesh.triangles[i].vertexIds);

            if (matches.length >= 2) {
                polygon.neighbours.push(navigationMesh.triangles[i]);
            }
        }
    }

    function buildTrianglesFromWavefront(obj: IWavefront): ITriangleInfo {

        let polygonId = 1;
        let nm: ITriangleInfo = {
            triangles: [],
            vertices: obj.vertices,
        };

        // Convert the faces into a custom format that supports more than 3 vertices
        for (let i = 0, len = obj.faces.length; i < len; i++) {
            let face = obj.faces[i];
            nm.triangles.push(triangleFromFace(polygonId++, face));
        }

        // Build a list of adjacent polygons
        for (let i = 0, len = nm.triangles.length; i < len; i++) {
            buildTriangleNeighbours(nm.triangles[i], nm);
        }

        return nm;
    }

    function buildTriangleInfo(obj: IWavefront): ITriangleInfo {
        computeCentroids(obj);
        mergeVertices(obj);
        return buildTrianglesFromWavefront(obj);
    }

    function roundNumber(number: number, decimals: number): number {
        let newnumber = new Number(number + '').toFixed(decimals);
        return parseFloat(newnumber);
    }

    function neighbourSpreadGroup(polygon: ITriangle): void {
        for (let i = 0, len = polygon.neighbours.length; i < len; i++) {
            const neighbour = polygon.neighbours[i];
            if (neighbour.group == undefined) {
                neighbour.group = polygon.group;
                neighbourSpreadGroup(neighbour);
            }
        }
    }

    function buildPolygonGroups(nm: ITriangleInfo): Array<Array<ITriangle>> {

        let polygonGroups: Array<Array<ITriangle>> = [];
        let groupId = 0;

        for (let i = 0, len = nm.triangles.length; i < len; i++) {
            const polygon = nm.triangles[i];

            if (polygon.group == undefined) {
                polygon.group = groupId++;
                neighbourSpreadGroup(polygon);
            }

            if (!polygonGroups[polygon.group]) polygonGroups[polygon.group] = [];

            polygonGroups[polygon.group].push(polygon);
        }

        return polygonGroups;
    }

    function getSharedVerticesInOrder(a: ITriangle, b: ITriangle) {

        let aList: number[] = a.vertexIds;
        let bList: number[] = b.vertexIds;

        let sharedVertices: number[] = [];

        for (let i = 0, len = aList.length; i < len; i++) {
            const vId = aList[i];
            if (bList.indexOf(vId) > -1) {
                sharedVertices.push(vId);
            }
        }

        if (sharedVertices.length < 2) return [];

        // console.log("TRYING aList:", aList, ", bList:", bList, ", sharedVertices:", sharedVertices);
        if (sharedVertices.indexOf(aList[0]) > -1 && sharedVertices.indexOf(aList[aList.length - 1]) > -1) {
            // Vertices on both edges are bad, so shift them once to the left
            aList.push(aList.shift());
        }

        if (sharedVertices.indexOf(bList[0]) > -1 && sharedVertices.indexOf(bList[bList.length - 1]) > -1) {
            // Vertices on both edges are bad, so shift them once to the left
            bList.push(bList.shift());
        }

        // Again!
        sharedVertices = [];

        for (let i = 0, len = aList.length; i < len; i++) {
            const vId = aList[i];
            if (bList.indexOf(vId) > -1) {
                sharedVertices.push(vId);
            }
        }

        return sharedVertices;
    }

    export interface IGroupedTriangleInfo {
        vertices: IVec3Like[];
        groups: Array<Array<ITriangle>>
    }

    function findPolygonIndex(group: ITriangle[], p: ITriangle) {
        for (let i = 0; i < group.length; i++) {
            if (p === group[i]) return i;
        }
    }

    function groupTriangleInfo(nm: ITriangleInfo): IGroupedTriangleInfo {

        let saveObj: IGroupedTriangleInfo = {
            vertices: null,
            groups: null
        };

        for (let i = 0, len = nm.vertices.length; i < len; i++) {
            let vertice = nm.vertices[i];
            vertice.x = roundNumber(vertice.x, 2)
            vertice.y = roundNumber(vertice.y, 2)
            vertice.z = roundNumber(vertice.z, 2)
        }

        saveObj.vertices = nm.vertices;

        let groups = buildPolygonGroups(nm);

        saveObj.groups = [];

        for (let i = 0, len = groups.length; i < len; i++) {
            const group = groups[i];
            let newGroup = [];

            for (let j = 0, len2 = group.length; j < len2; j++) {
                const p = group[j];

                let neighbours = [];
                let portals = [];

                for (let z = 0, len3 = p.neighbours.length; z < len3; z++) {
                    const n = p.neighbours[z];
                    neighbours.push(findPolygonIndex(group, n));
                    portals.push(getSharedVerticesInOrder(p, n));
                }

                p.centroid.x = roundNumber(p.centroid.x, 2);
                p.centroid.y = roundNumber(p.centroid.y, 2);
                p.centroid.z = roundNumber(p.centroid.z, 2);

                newGroup.push({
                    id: findPolygonIndex(group, p),
                    neighbours: neighbours,
                    vertexIds: p.vertexIds,
                    centroid: p.centroid,
                    portals: portals
                });
            }

            saveObj.groups.push(newGroup);
        }

        return saveObj;
    }

    function isPointInPoly(poly, pt) {
        for (var c = false, i = -1, l = poly.length, j = l - 1; ++i < l; j = i)
            ((poly[i].z <= pt.z && pt.z < poly[j].z) || (poly[j].z <= pt.z && pt.z < poly[i].z)) && (pt.x < (poly[j].x - poly[i].x) * (pt.z - poly[i].z) / (poly[j].z - poly[i].z) + poly[i].x) && (c = !c);
        return c;
    }

    function isVectorInPolygon(vector: IVec3Like, polygon, vertices: IVec3Like[]): boolean {

        // reference point will be the centroid of the polygon
        // We need to rotate the vector as well as all the points which the polygon uses

        let center = polygon.centroid;

        let lowestPoint = 100000;
        let highestPoint = -100000;

        let polygonVertices = [];

        for (let i = 0, len = polygon.vertexIds.length; i < len; i++) {
            const vId = polygon.vertexIds[i];
            lowestPoint = Math.min(vertices[vId].y, lowestPoint);
            highestPoint = Math.max(vertices[vId].y, highestPoint);
            polygonVertices.push(vertices[vId]);
        }

        if (vector.y < highestPoint + 0.5 && vector.y > lowestPoint - 0.5 && isPointInPoly(polygonVertices, vector)) {
            return true;
        }
        return false;
    }

    export function random(n: number, t: number = null) {
        return null == t && (t = n, n = 0), n + Math.floor(Math.random() * (t - n + 1))
    }

    export function buildNodesByJson(json: any) {
        let p2 = json.vertices;
        let ii = json.faces;
        let faces: IFace[] = [];
        for (let i = 0; i < ii.length / 5; i++) {
            faces.push(face(ii[i * 5 + 1], ii[i * 5 + 2], ii[i * 5 + 3]));
        };
        let p: IVec3Like[] = [];
        for (let i = 0; i < p2.length; i += 3) {
            p.push({ x: p2[i], y: p2[i + 1], z: p2[i + 2] });
        };

        const obj: IWavefront = {
            faces: faces,
            vertices: p
        };
        let zoneNodes = buildNodes(obj);
        return zoneNodes;
    }

    export class Channel {
        public portals: {
            left: IVec3Like,
            right: IVec3Like
        }[] = [];
        public path: IVec3Like[];

        public push(p1, p2?) {
            if (p2 === undefined) p2 = p1;
            this.portals.push({
                left: p1,
                right: p2
            });
        };

        public triarea2(a, b, c) {
            var ax = b.x - a.x;
            var az = b.z - a.z;
            var bx = c.x - a.x;
            var bz = c.z - a.z;
            return bx * az - ax * bz;
        }

        public vequal(a: IVec3Like, b: IVec3Like) {
            return Vec3.squaredDistance(a, b) < 0.00001;
        }

        public stringPull() {
            let portals = this.portals;
            let pts = [];
            // Init scan state
            let portalApex, portalLeft, portalRight;
            let apexIndex = 0,
                leftIndex = 0,
                rightIndex = 0;

            portalApex = portals[0].left;
            portalLeft = portals[0].left;
            portalRight = portals[0].right;

            // Add start point.
            pts.push(portalApex);

            for (let i = 1; i < portals.length; i++) {
                let left = portals[i].left;
                let right = portals[i].right;

                // Update right vertex.
                if (this.triarea2(portalApex, portalRight, right) <= 0.0) {
                    if (this.vequal(portalApex, portalRight) || this.triarea2(portalApex, portalLeft, right) > 0.0) {
                        // Tighten the funnel.
                        portalRight = right;
                        rightIndex = i;
                    } else {
                        // Right over left, insert left to path and restart scan from portal left point.
                        pts.push(portalLeft);
                        // Make current left the new apex.
                        portalApex = portalLeft;
                        apexIndex = leftIndex;
                        // Reset portal
                        portalLeft = portalApex;
                        portalRight = portalApex;
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;
                        // Restart scan
                        i = apexIndex;
                        continue;
                    }
                }

                // Update left vertex.
                if (this.triarea2(portalApex, portalLeft, left) >= 0.0) {
                    if (this.vequal(portalApex, portalLeft) || this.triarea2(portalApex, portalRight, left) < 0.0) {
                        // Tighten the funnel.
                        portalLeft = left;
                        leftIndex = i;
                    } else {
                        // Left over right, insert right to path and restart scan from portal right point.
                        pts.push(portalRight);
                        // Make current right the new apex.
                        portalApex = portalRight;
                        apexIndex = rightIndex;
                        // Reset portal
                        portalLeft = portalApex;
                        portalRight = portalApex;
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;
                        // Restart scan
                        i = apexIndex;
                        continue;
                    }
                }
            }

            if ((pts.length === 0) || (!this.vequal(pts[pts.length - 1], portals[portals.length - 1].left))) {
                // Append last point to path.
                pts.push(portals[portals.length - 1].left);
            }

            this.path = pts;
            return pts;
        }
    }

    export function buildNodes(obj: IWavefront) {
        let navigationMesh = buildTriangleInfo(obj);

        let zoneNodes = groupTriangleInfo(navigationMesh);

        // console.log('vertices');
        // for (let i = 0; i < zoneNodes.vertices.length; i++) {
        //     const element = zoneNodes.vertices[i];
        //     console.log('v', element.x, element.y, element.z);
        // }

        // console.log('groups');
        // for (let i = 0; i < zoneNodes.groups.length; i++) {
        //     const element = zoneNodes.groups[i];
        //     console.log('group', i);
        //     for (let j = 0; j < element.length; j++) {
        //         const e = element[j];
        //         console.log('f', e.vertexIds[0], e.vertexIds[1], e.vertexIds[2]);
        //     }
        // }

        return zoneNodes;
    }

    export class Store {
        static zoneNodes: { [key: string]: IGroupedTriangleInfo } = {};
    }

    export function setZoneData(zone: string, data: IGroupedTriangleInfo) {
        Store.zoneNodes[zone] = data;
    }

    export function getGroup(zone: string, position: IVec3Like): number {

        if (!Store.zoneNodes[zone]) return null;

        let closestNodeGroup = null;

        let distance = Math.pow(50, 2);

        for (let i = 0, len = Store.zoneNodes[zone].groups.length; i < len; i++) {
            const group = Store.zoneNodes[zone].groups[i];

            for (let j = 0, len2 = group.length; j < len2; j++) {
                const node = group[j];
                let measuredDistance = Vec3.squaredDistance(node.centroid, position);
                if (measuredDistance < distance) {
                    closestNodeGroup = i;
                    distance = measuredDistance;
                }
            }
        }
        return closestNodeGroup;
    }

    export function getRandomNode(zone: string, group: number, nearPosition: Vec3, nearRange: number): IVec3Like {

        if (!Store.zoneNodes[zone]) return { x: 0, y: 0, z: 0 };

        nearPosition = nearPosition || null;
        nearRange = nearRange || 0;

        let candidates = [];

        let polygons = Store.zoneNodes[zone].groups[group];

        for (let i = 0, len = polygons.length; i < len; i++) {
            const p = polygons[i];
            if (nearPosition && nearRange) {
                if (Vec3.squaredDistance(nearPosition, p.centroid) < nearRange * nearRange) {
                    candidates.push(p.centroid);
                }
            } else {
                candidates.push(p.centroid);
            }
        }

        if (candidates.length > 0) {
            let index = random(candidates.length);
            candidates[index]

            return candidates[index];
        }
    }

    export class Astar {

        public static init(graph: ITriangle[]) {
            for (let x = 0; x < graph.length; x++) {
                let node = graph[x];
                resetTriangle(graph[x]);

            }
        }

        public static cleanUp(graph: ITriangle[]) {
            for (let x = 0; x < graph.length; x++) {
                cleanTriangle(graph[x]);
            }
        }

        public static heap() {
            return new Heap<ITriangle>();
        }

        public static search(graph: ITriangle[], start: ITriangle, end: ITriangle) {
            Astar.init(graph);

            let openHeap = Astar.heap();

            openHeap.push(start);

            while (openHeap.size > 0) {

                // Grab the lowest f(x) to process next.  Heap keeps this sorted for us.
                let currentNode = openHeap.pop();

                // End case -- result has been found, return the traced path.
                if (currentNode === end) {
                    let curr = currentNode;
                    let ret = [];
                    while (curr.parent) {
                        ret.push(curr);
                        curr = curr.parent;
                    }
                    this.cleanUp(ret);
                    return ret.reverse();
                }

                // Normal case -- move currentNode from open to closed, process each of its neighbours.
                currentNode.closed = true;

                // Find all neighbours for the current node. Optionally find diagonal neighbours as well (false by default).
                let neighbours = Astar.neighbours(graph, currentNode);

                for (let i = 0, il = neighbours.length; i < il; i++) {
                    let neighbour = neighbours[i];

                    if (neighbour.closed) {
                        // Not a valid node to process, skip to next neighbour.
                        continue;
                    }

                    // The g score is the shortest distance from start to current node.
                    // We need to check if the path we have arrived at this neighbour is the shortest one we have seen yet.
                    let gScore = currentNode.g + neighbour.cost;
                    let beenVisited = neighbour.visited;

                    if (!beenVisited || gScore < neighbour.g) {

                        // Found an optimal (so far) path to this node.  Take score for node to see how good it is.
                        neighbour.visited = true;
                        neighbour.parent = currentNode;
                        if (!neighbour.centroid || !end.centroid) debugger;
                        neighbour.h = neighbour.h || Astar.heuristic(neighbour.centroid, end.centroid);
                        neighbour.g = gScore;
                        neighbour.f = neighbour.g + neighbour.h;

                        if (!beenVisited) {
                            // Pushing to heap will put it in proper place based on the 'f' value.
                            openHeap.push(neighbour);
                        } else {
                            // Already seen the node, but since it has been rescored we need to reorder it in the heap
                            openHeap.updateItem(neighbour);
                        }
                    }
                }
            }

            // No result was found - empty array signifies failure to find path.
            return [];
        }

        public static heuristic(pos1, pos2) {
            return Vec3.squaredDistance(pos1, pos2);
        }

        public static neighbours(graph, node) {
            let ret = [];

            for (let e = 0; e < node.neighbours.length; e++) {
                ret.push(graph[node.neighbours[e]]);
            }
            return ret;
        }
    }

    export function findPath(startPosition: IVec3Like, targetPosition: IVec3Like, zone: string, group: number): IVec3Like[] {
        let allNodes = Store.zoneNodes[zone].groups[group];
        let vertices = Store.zoneNodes[zone].vertices;

        let closestNode = null;
        let distance = Math.pow(50, 2);

        for (let i = 0, len = allNodes.length; i < len; i++) {
            const node = allNodes[i];
            let measuredDistance = Vec3.squaredDistance(node.centroid, startPosition);
            if (measuredDistance < distance) {
                closestNode = node;
                distance = measuredDistance;
            }
        }
        let farthestNode = null;
        distance = Math.pow(50, 2);

        for (let i = 0, len = allNodes.length; i < len; i++) {
            const node = allNodes[i];
            let measuredDistance = Vec3.squaredDistance(node.centroid, targetPosition);
            if (measuredDistance < distance && isVectorInPolygon(targetPosition, node, vertices)) {
                farthestNode = node;
                distance = measuredDistance;
            }
        }

        // If we can't find any node, just go straight to the target
        if (!closestNode || !farthestNode) {
            return null;
        }

        let paths = Astar.search(allNodes, closestNode, farthestNode);

        let getPortalFromTo = function (a, b) {
            for (let i = 0; i < a.neighbours.length; i++) {
                if (a.neighbours[i] === b.id) {
                    return a.portals[i];
                }
            }
        };

        // We got the corridor
        // Now pull the rope

        let channel = new Channel();

        channel.push(startPosition);

        for (let i = 0; i < paths.length; i++) {
            let polygon = paths[i];

            let nextPolygon = paths[i + 1];

            if (nextPolygon) {
                let portals = getPortalFromTo(polygon, nextPolygon);
                channel.push(
                    vertices[portals[0]],
                    vertices[portals[1]]
                );
            }

        }

        channel.push(targetPosition);
        channel.stringPull();

        let threeVectors = [...channel.path];

        // We don't need the first one, as we already know our start position
        threeVectors.shift();

        return threeVectors;
    }
}



export default NavMesh;
