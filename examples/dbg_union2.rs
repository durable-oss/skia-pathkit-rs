use pathkit::core::{Path, Rect, Verb, Point};
use pathkit::pathops::{op, PathOp};
fn contours(p:&Path)->usize{ p.verbs().iter().filter(|v| matches!(v, Verb::Move)).count() }
// polygonal approximation of a disc: no conics involved
fn ngon(cx:f32,cy:f32,r:f32,n:usize)->Path{
    let mut p=Path::new();
    for i in 0..n {
        let a = i as f32 / n as f32 * std::f32::consts::TAU;
        let (x,y)=(cx+r*a.cos(), cy+r*a.sin());
        if i==0 { p.move_to(x,y); } else { p.line_to(x,y); }
    }
    p.close(); p
}
fn rect(l:f32,t:f32,r:f32,b:f32)->Path{ let mut p=Path::new(); p.add_rect_simple(Rect::from_ltrb(l,t,r,b)); p }
fn main(){
    println!("-- ngon unions (NO conics) --");
    for off in [0.5f32,1.0,5.0,20.0,60.0,100.0] {
        let a=ngon(200.0,200.0,40.0,64); let b=ngon(200.0+off,200.0,40.0,64);
        let u=op(&a,&b,PathOp::Union).unwrap();
        println!("ngon off={off:6} -> {} contours empty={}", contours(&u), u.is_empty());
    }
    println!("-- rect tangency / near-coincidence --");
    for off in [0.0f32,0.5,1.0,9.0,10.0,11.0] {
        let a=rect(0.0,0.0,10.0,10.0); let b=rect(off,0.0,off+10.0,10.0);
        let u=op(&a,&b,PathOp::Union).unwrap();
        println!("rect off={off:5} -> {} contours empty={} bounds={:?}", contours(&u), u.is_empty(), u.bounds());
    }
}
