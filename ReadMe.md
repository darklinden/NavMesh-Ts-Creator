# Cocos Creator 可用的 NavMesh 寻路, Creator 3.4.2 测试可用

## 相关工程

* <https://github.com/nickjanssen/PatrolJS>
* <https://github.com/lear315/NevMeshJSDemo>

## 工作流

* 在 unity 中 bake NavMesh 并导出
* 将 fbx 或 obj 使用 blender 进行处理
* 使用 python/convert_obj_three.py 导出 json
* 在 creator 工程中使用 json

## 问题

* unity 自动生成的 NavMesh 由于是三角形构成，地面高度很可能高低不平，导致角色寻路播放时会 “漂浮” 或 “钻地”, 建议不使用原模型而另建减免模型生成 NavMesh
