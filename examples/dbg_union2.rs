use pathkit::core::{Path, Verb};
use pathkit::pathops::{op, PathOp};
fn contours(p:&Path)->usize{ p.verbs().iter().filter(|v| matches!(v, Verb::Move)).count() }
fn ngon(cx:f32,cy:f32,r:f32,n:usize)->Path{
    let mut p=Path::new();
    for i in 0..n {
        let a = i as f32 / n as f32 * std::f32::consts::TAU;
        let (x,y)=(cx+r*a.cos(), cy+r*a.sin());
        if i==0 { p.move_to(x,y); } else { p.line_to(x,y); }
    }
    p.close(); p
}
fn main(){
    println!("vary vertex count, r=40 off=5:");
    for n in [4usize,6,8,12,16,24,32,48,64,128] {
        let a=ngon(200.0,200.0,40.0,n); let b=ngon(205.0,200.0,40.0,n);
        let u=op(&a,&b,PathOp::Union).unwrap();
        println!("  n={n:4} -> {} contours empty={}", contours(&u), u.is_empty());
    }
    println!("vary offset, n=64 r=40  (edge len ~ {:.3}):", 2.0*40.0*(std::f32::consts::PI/64.0).sin());
    for off in [0.5f32,2.0,4.0,6.0,8.0,10.0,15.0,20.0] {
        let a=ngon(200.0,200.0,40.0,64); let b=ngon(200.0+off,200.0,40.0,64);
        let u=op(&a,&b,PathOp::Union).unwrap();
        println!("  off={off:5} -> {} contours empty={}", contours(&u), u.is_empty());
    }
    println!("vary radius at n=64, off=5 (scale the whole thing):");
    for r in [10.0f32,40.0,100.0,400.0,1000.0] {
        let a=ngon(2000.0,2000.0,r,64); let b=ngon(2000.0+r/8.0,2000.0,r,64);
        let u=op(&a,&b,PathOp::Union).unwrap();
        println!("  r={r:6} edge={:.3} -> {} contours empty={}", 2.0*r*(std::f32::consts::PI/64.0).sin(), contours(&u), u.is_empty());
    }
}
