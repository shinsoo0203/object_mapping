
"use strict";

let BoundingBox = require('./BoundingBox.js');
let BoundingBoxes = require('./BoundingBoxes.js');
let ObjectArray = require('./ObjectArray.js');
let ObjectCount = require('./ObjectCount.js');
let ObjectPoint = require('./ObjectPoint.js');
let CheckForObjectsGoal = require('./CheckForObjectsGoal.js');
let CheckForObjectsActionFeedback = require('./CheckForObjectsActionFeedback.js');
let CheckForObjectsActionGoal = require('./CheckForObjectsActionGoal.js');
let CheckForObjectsAction = require('./CheckForObjectsAction.js');
let CheckForObjectsResult = require('./CheckForObjectsResult.js');
let CheckForObjectsFeedback = require('./CheckForObjectsFeedback.js');
let CheckForObjectsActionResult = require('./CheckForObjectsActionResult.js');

module.exports = {
  BoundingBox: BoundingBox,
  BoundingBoxes: BoundingBoxes,
  ObjectArray: ObjectArray,
  ObjectCount: ObjectCount,
  ObjectPoint: ObjectPoint,
  CheckForObjectsGoal: CheckForObjectsGoal,
  CheckForObjectsActionFeedback: CheckForObjectsActionFeedback,
  CheckForObjectsActionGoal: CheckForObjectsActionGoal,
  CheckForObjectsAction: CheckForObjectsAction,
  CheckForObjectsResult: CheckForObjectsResult,
  CheckForObjectsFeedback: CheckForObjectsFeedback,
  CheckForObjectsActionResult: CheckForObjectsActionResult,
};
