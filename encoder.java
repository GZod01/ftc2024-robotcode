DcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // tare the encoder
DcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// Position droite = 8 ticks
// position initiale = 0 ticks
// position allongée = - 16 ticks
// in opMode:
int pos_to_go = ...; // position to go
DcMotor.setTargetPosition(pos_to_go); // set target position, the motor will run to this pos
DcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // mode run to position
