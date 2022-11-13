/* This file contains the data / context code of the COBPjs application (data-access layer).
It consists mainly of 
the initial context (ctx.populateContext(entities[])), 
queries (ctx.registerQuery(name, function(entity){}))
and effects (ctx.registerEffect(name, function(data){})).
*/

/* constants of the angle values of the distances that are stored in the context
as well as the minimum allowed distance to obstacles
*/
const FRONT = 0;
const FRONTLEFT = 30;
const FRONTRIGHT = 330;
const LEFT = 60;
const RIGHT = 300;
const BACK = 180;
const minimum_forward_dist = 0.5;
const minimum_side_dist = 0.4;

// creates the initial context of the application by adding context-entities to it
ctx.populateContext([
  ctx.Entity("bot1", "robot", { oAhead: 3, oFrontLeft: 3, oFrontRight: 3, oLeft: 3, oRight: 3, oBack: 3, 
    x_coor: 0, y_coor: 0, orientation: 0}),
  ctx.Entity("battery1", "battery", {battery_charge: 800, battery_capacity: 800, hub_x_coor: -1.5, hub_y_coor: -0.5, charging: false}),
  ctx.Entity("ts1", "target_store", {target: null}),
  ctx.Entity("deliveryQueue1", "deliveryQueue", {queueList: [
    {source: {x_coor: 2.0, y_coor: 0.5}, goal: {x_coor: -2.0, y_coor: 0.0}},
    {source: {x_coor: 0.0, y_coor: -2.0}, goal: {x_coor: -2.0, y_coor: 0.0}},
    {source: {x_coor: -0.5, y_coor: 2.0}, goal: {x_coor: -2.0, y_coor: 0.0}},
  ]})
]);

/* This is a query. It consists of a name that is used to bind CBTs to it and 
a function that takes a entity as an parameter and returns true if the entity 
fulfills the query and false otherwise.
Queries are run at the start of the application and at synchronization if an 
effect was triggered.

This query checks if the context contains an entity with the type "robot". 
Since this is the case a Live-Copy is spawned for every CBT that is bound to 
this query. The context entity becomes the seed of the Live-Copy and is 
accessible through the entity parameter of its function. 
*/
ctx.registerQuery("Robot", function (entity) {
  return entity.type == "robot";
});

/* This query checks whether there is an obstacle ahead that is closer than the 
minimum distance. If this is the case a Live-Copy of the "avoid walls ahead"-CBT 
is spawned that prevents forward movement.  
*/
ctx.registerQuery("ObstacleAhead", function (entity) {
  return entity.type == "robot" && entity.oAhead < minimum_forward_dist;
});

ctx.registerQuery("ObstacleLeft", function (entity) {
  return entity.type == "robot" && entity.oLeft < minimum_side_dist;
});

ctx.registerQuery("ObstacleRight", function (entity) {
  return entity.type == "robot" && entity.oRight < minimum_side_dist; 
});

ctx.registerQuery("ObstacleFrontLeft", function (entity) {
  return entity.type == "robot" && entity.oFrontLeft < minimum_side_dist;
});

ctx.registerQuery("ObstacleFrontRight", function (entity) {
  return entity.type == "robot" && entity.oFrontRight < minimum_side_dist; 
});

ctx.registerQuery("ObstacleBehind", function (entity) {
  return entity.type == "robot" && entity.oBack < minimum_side_dist; 
});

/* checks if any obstacle needs to be avoided and spawns Live-Copies that 
initiate the movement during obstacle avoidance and block the movements 
toward the target
*/
ctx.registerQuery("Obstacle", function (entity) {
  return entity.type == "robot" && (entity.oAhead < minimum_forward_dist || 
                                    entity.oFrontLeft < minimum_side_dist || 
                                    entity.oFrontRight < minimum_side_dist ||
                                    entity.oLeft < minimum_side_dist || 
                                    entity.oRight < minimum_side_dist); 
});

ctx.registerQuery("Target", function (entity) {
  return entity.type == "target";
});

ctx.registerQuery("DeliveryQueue", function (entity) {
  return entity.type == "deliveryQueue";
});

// checks if the delivery queue actually contains entries
ctx.registerQuery("Delivery", function (entity) {
  return entity.type == "deliveryQueue" && entity.queueList.length > 0;
});

/* Checks if the battery charge falls below a certain threshold. 
The charging attribute is needed because the query would fail once the battery is 
charged past the threshold and the Live-Copy would be terminated before the charging 
process is complete, 
*/
ctx.registerQuery("Battery", function (entity) {
  return entity.type == "battery" && (entity.battery_charge < 200 || entity.charging == true);
});

ctx.registerQuery("TargetStore", function (entity) {
  return entity.type == "target_store";
});

/* This is an effect. It is executed when an event with the name of that is 
specified as its first argument is selected and can access the data of this 
event through the data parameter of its function.

This effect reacts to the selection of a "ScanData"-event and adds its data 
to the context by updating the robot entity. 
*/
ctx.registerEffect("ScanData", function (data) {
  function getDistanceToObstacle(range, range_max) {
    // max range is 3
    return range == null ? range_max : range;
  }

  var ranges = data.ranges;

  /* Updates the distance data of the robot with the data of the scan. 
  Note: The robot id ("bot1") is hardcoded into the method call. This was done 
  because the application only uses one robot that is always active and because 
  it would require to pass the robot entity with the data of all events that 
  change or access its attributes. This would be especially bothersome for 
  events that are requested by Live-Copies that do not have the robot entity as 
  seed.   
  */
  ctx.getEntityById("bot1").oAhead = getDistanceToObstacle(ranges[FRONT], data.range_max);
  ctx.getEntityById("bot1").oFrontLeft = getDistanceToObstacle(ranges[FRONTLEFT], data.range_max);
  ctx.getEntityById("bot1").oFrontRight = getDistanceToObstacle(ranges[FRONTRIGHT], data.range_max);
  ctx.getEntityById("bot1").oLeft = getDistanceToObstacle(ranges[LEFT], data.range_max);
  ctx.getEntityById("bot1").oRight = getDistanceToObstacle(ranges[RIGHT], data.range_max);
  ctx.getEntityById("bot1").oBack = getDistanceToObstacle(ranges[BACK], data.range_max);
})

ctx.registerEffect("PositionData", function(data) {
  ctx.getEntityById("bot1").x_coor = data.x_coor;
  ctx.getEntityById("bot1").y_coor = data.y_coor;
  ctx.getEntityById("bot1").orientation = data.orientation;
  
  // runs the query of the name "Target" and receives its results (all targets in the context) 
  var assigned_target = ctx.runQuery("Target");
  
  // updates all targets in the context (there should only be one target at a time)
  assigned_target.forEach(function(entity) {
    entity.rob_x_coor = data.x_coor;
    entity.rob_y_coor = data.y_coor;
    entity.rob_orientation = data.orientation;
  });
});

ctx.registerEffect("SetTarget", function(data) {
  var target_assigned = ctx.runQuery("Target");
  
  // removes previous target(s) if they exist (should actually never be the case)
  target_assigned.forEach(function(entity) {
    ctx.removeEntity(entity);
  });
  
  var bot = ctx.getEntityById("bot1")

  // creates a new target entity and inserts it into the context
  ctx.insertEntity(ctx.Entity(data.id , "target", {
    tar_x_coor: data.x_coor, 
    tar_y_coor: data.y_coor, 
    rob_x_coor: bot.x_coor, 
    rob_y_coor: bot.y_coor, 
    rob_orientation: bot.orientation}));
});

// removes the target from the context once it has been reached
ctx.registerEffect("TargetReached", function(data) {
  ctx.removeEntity(data);
});

// removes the first order of the delivery queue once it has been completed
ctx.registerEffect("DeliveryComplete", function(data) {
  ctx.getEntityById(data).queueList.shift();
});

// simulates the battery by decreasing its charge at every movement 
ctx.registerEffect("UpdateVelocity", function(data) {
  ctx.getEntityById("battery1").battery_charge--;
});

// simulates the charging of the battery
ctx.registerEffect("ChargeBattery", function(data) {
  ctx.getEntityById(data).battery_charge++;
  ctx.getEntityById(data).charging = true;
});

/* Sets the charging attribute to false once the charging is complete 
 which will cause the "Battery"-query to fail and thus terminate the Live-Copy. 
 */
ctx.registerEffect("ChargingComplete", function(data) {
  ctx.getEntityById(data).charging = false;
});

// puts the current target into the store entity and removes it from the context
ctx.registerEffect("PutTargetInStore", function(data) {
  var targets = ctx.runQuery("Target");
  var cur_tar = targets.length > 0 ? targets[0] : null

  ctx.getEntityById(data.storeId).target = cur_tar;

  targets.forEach(function(entity) {
    ctx.removeEntity(entity);
  });
});

// restores the target from the store (if it exists)
ctx.registerEffect("LoadTargetFromStore", function(data) {
  var target = ctx.getEntityById(data.storeId).target
  
  if(target != null){
    ctx.insertEntity(target);
  }
});

// adds a new delivery order to the end of the delivery queue
ctx.registerEffect("EnqueueDelivery", function(data) {
  ctx.getEntityById(data.delivery_list).queueList.push(data.delivery);
});
