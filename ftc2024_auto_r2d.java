package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Ftc2024_autonome_api;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ftc2024_auto_r2d extends LinearOpMode{
    //hi
    public Ftc2024_autonome_api.AutoMode autonomous_mode = Ftc2024_autonome_api.AutoMode.R2D;
    
    @Override
    public void runOpMode() {
        Ftc2024_autonome_api a = new Ftc2024_autonome_api();
        a.hardwareMap = this.hardwareMap;
        a.runOpMode();
    }
}
       