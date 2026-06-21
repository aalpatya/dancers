# Modifying DANCeRS

How to extend the simulator. See [How it works](how_it_works.md) for the core concepts first.

> These instruction pages are a work in progress and will keep being updated — check back for more.

## Adding your own factor-graph layer (sub-problem)

A **layer** is one GBP sub-problem a robot solves (planning, consensus, …). You can stack as many as you like, in any order, each with its own name. Adding one takes **3 steps**:

> **Easiest start:** copy [`ExampleLayer.h`](../inc/CustomFactorGraphLayers/ExampleLayer.h) / [`ExampleLayer.cpp`](../src/CustomFactorGraphLayers/ExampleLayer.cpp) — a compile-checked, fill-in-the-`TODO`s template covering everything below — then rename and edit.

1. **Write the layer class.**

   Create `inc/CustomFactorGraphLayers/MyLayer.h` and `src/CustomFactorGraphLayers/MyLayer.cpp` (the `.cpp` is compiled automatically — no CMake edit). Subclass `FactorGraphLayer` and override the hooks you need (all default to no-ops). Read tuning from the config bag in the constructor — `cfg.params.numf("MY_SIGMA", 0.1f)` — so you never touch `Globals`. Custom factors/variables subclass `Factor`/`Variable` (see [`inc/CustomFactorGraphLayers/Pathplanning.h`](../inc/CustomFactorGraphLayers/Pathplanning.h) for the pattern).

   <details>
   <summary><b>Write these functions</b></summary>

   | Hook | When it runs | What it does |
   |------|--------------|--------------|
   | `initialiseLayerNodes` | once, at robot creation | create this layer's variables and factors |
   | `createInterrobotFactors` | when two robots become neighbours | create factors between this robot and another |
   | `preGBPUpdateNodes` | each timestep, before GBP | e.g. add/remove nodes |
   | `postGBPUpdateNodes` | each timestep, after GBP | e.g. read the belief back out |
   </details>


   <details>
   <summary><b>(Optional) Draw your layer</b></summary>

   A layer can render itself by *also* inheriting [`DrawableLayer`](../inc/CustomFactorGraphLayers/DrawableLayer.h) and implementing `draw(Robot* robot, Simulator* sim, Color& col)`. Each frame `Robot::draw()` ([`src/Robot.cpp`](../src/Robot.cpp)) `dynamic_cast`s every layer to `DrawableLayer` and calls `draw()` on the ones that are — so a layer opts in just by inheriting it, no registration needed. `col` is the robot's running display colour: read it to tint your geometry, or write to it to recolour the robot model itself (e.g. the consensus layer tints each robot by its current decision).

   ```cpp
   // MyLayer.h — additionally inherit DrawableLayer
   #include <CustomFactorGraphLayers/DrawableLayer.h>

   class MyLayer : public FactorGraphLayer, public DrawableLayer {
   public:
       // ...constructor + lifecycle hooks...
       void draw(Robot* robot, Simulator* sim, Color& col) override;
   };
   ```

   ```cpp
   // MyLayer.cpp
   void MyLayer::draw(Robot* robot, Simulator* sim, Color& col){
       for (auto& [vkey, var] : variables_) var->draw(col);              // draw this layer's variables
       // col = decisionColor(robot->decision_, globals.NUM_DECISIONS);  // (optional) recolour the robot model
   }
   ```
   </details>

2. **List it in the factory** — one line in `makeLayer()` ([`src/Robot.cpp`](../src/Robot.cpp)), mapping the config `type` string to your class:

   ```cpp
   if (type == "mylayer") return std::make_shared<MyLayer>(graph_id, lid, cfg);
   ```

3. **Add it to a config** under `FACTORGRAPH_LAYERS`:

   ```yaml
     - name: MyLayer
       type: mylayer
       params:
         MY_SIGMA: 0.1
   ```

That's it — the layer is built for every robot and stepped by GBP.

## Custom factors and variables

A layer's variables and factors are usually your own subclasses of `Variable` and `Factor`. A factor
subclass implements `computeResidualAndJacobian(X)` and forwards its group name, observation `z`, and
sigma to the base `Factor`; see [Creating a custom factor](how_it_works.md#creating-a-custom-factor)
for the full pattern (including the `jacobianFirstOrder` finite-difference helper). Worked examples:
the planning factors in [`Pathplanning.cpp`](../src/CustomFactorGraphLayers/Pathplanning.cpp), and the
fill-in-the-`TODO`s `ExampleFactor`/`ExampleVariable` in
[`ExampleLayer.cpp`](../src/CustomFactorGraphLayers/ExampleLayer.cpp).
