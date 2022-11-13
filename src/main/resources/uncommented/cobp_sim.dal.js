const FRONT = 0;
const FRONTLEFT = 30;
const FRONTRIGHT = 330;
const LEFT = 60;
const RIGHT = 300;
const BACK = 180;
const minimum_forward_dist = 0.5;
const minimum_side_dist = 0.4;

ctx.populateContext([
  ctx.Entity("bot1", "robot", { oAhead: 3, oFrontLeft: 3, oFrontRight: 3, oLeft: 3, oRight: 3, oBack: 3, 
    x_coor: 0, y_coor: 0, orientation: 0, 
    battery: 1}),
  ctx.Entity("battery1", "battery", {battery_charge: 800, battery_capacity: 800, hub_x_coor: -1.5, hub_y_coor: -0.5, charging: false}),
  ctx.Entity("ts1", "target_store", {target: null}),
  ctx.Entity("deliveryQueue1", "deliveryQueue", {queueList: [
    {source: {x_coor: 2.0, y_coor: 0.5}, goal: {x_coor: -2.0, y_coor: 0.0}},
    {source: {x_coor: 0.0, y_coor: -2.0}, goal: {x_coor: -2.0, y_coor: 0.0}},
    {source: {x_coor: -0.5, y_coor: 2.0}, goal: {x_coor: -2.0, y_coor: 0.0}},
  ]})
]);


ctx.registerQuery("Robot", function (entity) {
  return entity.type == "robot";
});

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

ctx.registerQuery("Delivery", function (entity) {
  return entity.type == "deliveryQueue" && entity.queueList.length > 0;
});


ctx.registerQuery("Battery", function (entity) {
  return entity.type == "battery" && (entity.battery_charge < 200 || entity.charging == true);
});

ctx.registerQuery("TargetStore", function (entity) {
  return entity.type == "target_store";
});


ctx.registerEffect("ScanData", 
  function (data) {
    function getDistanceToObstacle(range, range_max) {
      return range == null ? range_max : range;
    }

    var ranges = data.ranges;

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
  
  var assigned_target = ctx.runQuery("Target");
  
  assigned_target.forEach(function(entity) {
    entity.rob_x_coor = data.x_coor;
    entity.rob_y_coor = data.y_coor;
    entity.rob_orientation = data.orientation;
  });

});

ctx.registerEffect("SetTarget", function(data) {
  var target_assigned = ctx.runQuery("Target");
  
  target_assigned.forEach(function(entity) {
    ctx.removeEntity(entity);
  });
  
  var bot = ctx.getEntityById("bot1")

  ctx.insertEntity(ctx.Entity(data.id , "target", {
    tar_x_coor: data.x_coor, 
    tar_y_coor: data.y_coor, 
    rob_x_coor: bot.x_coor, 
    rob_y_coor: bot.y_coor, 
    rob_orientation: bot.orientation}));
});

ctx.registerEffect("TargetReached", function(data) {
  ctx.removeEntity(data);
});

ctx.registerEffect("DeliveryComplete", function(data) {
  ctx.getEntityById(data).queueList.shift();
});

ctx.registerEffect("UpdateVelocity", function(data) {
  ctx.getEntityById("battery1").battery_charge--;
});

ctx.registerEffect("ChargeBattery", function(data) {
  ctx.getEntityById(data).battery_charge++;
  ctx.getEntityById(data).charging = true;
});

ctx.registerEffect("ChargingComplete", function(data) {
  ctx.getEntityById(data).charging = false;
});

ctx.registerEffect("PutTargetInStore", function(data) {
  var targets = ctx.runQuery("Target");
  var cur_tar = targets.length > 0 ? targets[0] : null

  ctx.getEntityById(data.storeId).target = cur_tar;

  targets.forEach(function(entity) {
    ctx.removeEntity(entity);
  });
});

ctx.registerEffect("LoadTargetFromStore", function(data) {
  var target = ctx.getEntityById(data.storeId).target
  
  if(target != null){
    ctx.insertEntity(target);
  }
});

ctx.registerEffect("EnqueueDelivery", function(data) {
  ctx.getEntityById(data.delivery_list).queueList.push(data.delivery);
});
