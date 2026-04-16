
"use strict";

let SetSpeed = require('./SetSpeed.js')
let TorqueEnable = require('./TorqueEnable.js')
let StartController = require('./StartController.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')
let StopController = require('./StopController.js')
let RestartController = require('./RestartController.js')

module.exports = {
  SetSpeed: SetSpeed,
  TorqueEnable: TorqueEnable,
  StartController: StartController,
  SetCompliancePunch: SetCompliancePunch,
  SetComplianceSlope: SetComplianceSlope,
  SetComplianceMargin: SetComplianceMargin,
  SetTorqueLimit: SetTorqueLimit,
  StopController: StopController,
  RestartController: RestartController,
};
