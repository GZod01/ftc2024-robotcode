import os
file_exists = (lambda p: (os.path.isfile(p) and os.path.exists(p)))
modes_list_str="b4d.b2d.r2d.r4d.r2n.r4n.b4n.b2n"
modes_list = modes_list_str.split(".")
for m in modes_list:
   if not file_exists("ftc2024_auto_"+m.lower()+".java"):
       f= open("ftc2024_auto_"+m.lower()+".java","w")
       f.write("""
       package org.firstinspires.ftc.teamcode;
       import org.firstinspires.ftc.teamcode.Ftc2024_autonome_api;
       public class ftc2024_auto_{0} extends Ftc2024_autonome_api{{
           public AutoMode autonomous_mode = AutoMode.{1};
       }}
       """.format(m.lower(),m.upper()));