use safe_drive::{
    context::Context, error::DynError, logger::Logger, msg::common_interfaces::geometry_msgs::msg::Twist, topic::{subscriber, publisher::Publisher},
};

use safe_drive::msg::common_interfaces::geometry_msgs::msg;
use drobo_interfaces::msg::MdLibMsg;
use std::f64::consts::PI;


struct Tire{
    id:usize,
    raito:f64
}

struct  Chassis {
    fl:Tire,
    fr:Tire,
    br:Tire, 
    bl:Tire,
}

const CHASSIS:Chassis = Chassis{
    fl:Tire{
        id:0,
        raito:1.
    },
    fr:Tire{
        id:1,
        raito:1.
    },
    br:Tire{
        id:2,
        raito:1.
    },
    bl:Tire{
        id:3,
        raito:1.
    }
};



// const OMNI_DIA:f64 =  0.1;
const  MAX_PAWER_INPUT:f64 = 160.;
const  MAX_PAWER_OUTPUT:f64 = 999.;
const  MAX_REVOLUTION:f64 = 5400.;

fn main() -> Result<(), DynError>{

    // for debug
    let _logger = Logger::new("four_wheeled_control");


    let ctx = Context::new()?;
    let node = ctx.create_node("four_wheeled_control", None, Default::default())?;
    let subscriber = node.create_subscriber::<msg::Twist>("cmd_vel", None)?;
    let publisher = node.create_publisher::<drobo_interfaces::msg::MdLibMsg>("md_driver_topic", None)?;
    let mut selector = ctx.create_selector()?;
    
    selector.add_subscriber(
        subscriber,
        {
            Box::new(move |msg| {
                // pr_info!(logger, "receive: {:?}", msg.linear);
                let topic_callback_data = topic_callback(msg);
                // safe_drive::pr_info!(logger,"an:{},pa:{}",topic_callback_data[0],topic_callback_data[1]);
                move_chassis(topic_callback_data[0],topic_callback_data[1],topic_callback_data[2],&publisher);
            })
        },
    );

    loop {
        selector.wait()?;
    }
}


fn topic_callback(msg: subscriber::TakenMsg<Twist>) -> [f64;3]{
    // for debug
    let _logger = Logger::new("four_wheeled_control");
    
    let theta:f64 = msg.linear.y.atan2(-msg.linear.x);
    let pawer:f64 = (msg.linear.x.powf(2.) + msg.linear.y.powf(2.)).sqrt().min(MAX_PAWER_INPUT);
    

    [theta,pawer,msg.angular.z]
}

fn move_chassis(_theta:f64, _pawer:f64, _revolution:f64,publisher:&Publisher<MdLibMsg>){

    // for debug
    let _logger = Logger::new("four_wheeled_control");


    let mut motor_power:[f64;4] = [0.;4];
   
    motor_power[CHASSIS.fr.id] =  -MAX_PAWER_OUTPUT*(_revolution/MAX_REVOLUTION);
    motor_power[CHASSIS.br.id] =  -MAX_PAWER_OUTPUT*(_revolution/MAX_REVOLUTION);
    motor_power[CHASSIS.fl.id] =   MAX_PAWER_OUTPUT*(_revolution/MAX_REVOLUTION);
    motor_power[CHASSIS.bl.id] =   MAX_PAWER_OUTPUT*(_revolution/MAX_REVOLUTION);


    
    safe_drive::pr_info!(_logger,"fl : {} fr : {} br : {} bl : {} PA : {} ø : {} re :{}",
    motor_power[CHASSIS.fr.id],
    motor_power[CHASSIS.fl.id],
    motor_power[CHASSIS.br.id],
    motor_power[CHASSIS.bl.id],
    _pawer,
    _theta/PI*180.,
    _revolution
    );

    

    for i in 0..motor_power.len() {

        motor_power[i] = MAX_PAWER_OUTPUT * (_pawer/MAX_PAWER_INPUT) + motor_power[i];


        motor_power[i] = motor_power[i].max(-MAX_PAWER_OUTPUT);
        motor_power[i] = motor_power[i].min(MAX_PAWER_OUTPUT);
        
        send_pwm(i as u32,0,motor_power[i]>0., motor_power[i].abs() as u32,publisher);
    }

}

fn send_pwm(_address:u32, _semi_id:u32,_phase:bool,_power:u32,publisher:&Publisher<MdLibMsg>){
    let mut msg = drobo_interfaces::msg::MdLibMsg::new().unwrap();
    msg.address = _address as u8;
    msg.semi_id = _semi_id as u8;
    msg.mode = 2 as u8; //MotorLibのPWMモードに倣いました
    msg.phase = _phase as bool;
    msg.power = _power as u16;

    publisher.send(&msg).unwrap()

}