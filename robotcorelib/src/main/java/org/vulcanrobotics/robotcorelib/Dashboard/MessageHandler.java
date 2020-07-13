package org.vulcanrobotics.robotcorelib.Dashboard;

public class MessageHandler {

    public static void parseMessage(String msg) {

        String key = "";
        String val = "";
        if(msg.startsWith("/")) {
            String[] placeholder = msg.split(" ");
            key = placeholder[0];
            if(key.equals("/set")) {
//                String path;
                String type = placeholder[1];
                String id = placeholder[2];
                String cmd = placeholder[3];
                String num = placeholder[4];
                switch (type) {
                    case "DcMotor":
                        switch (cmd) {
                            case "limLow":
                                Dashboard.getMotors().get(Integer.parseInt(id)).setLowLimit(Integer.parseInt(num));
                                break;
                            case "limHigh":
                                Dashboard.getMotors().get(Integer.parseInt(id)).setHighLimit(Integer.parseInt(num));
                                break;
                            case "power":
                                Dashboard.getMotors().get(Integer.parseInt(id)).setPower(Double.parseDouble(num));
                                break;
                        }
                        break;
                    case "Servo":

                }
            }
            else if(key.equals("/start")) {
                Dashboard.running = true;
            }


        }
        else {
//            telemetry stuff
        }

    }

}
