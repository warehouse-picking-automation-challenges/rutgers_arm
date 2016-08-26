%%Please norice quatronian is w x y z
tf_right_pos = [0.418213957078 -0.216069349496 0.461213871052];
tf_right_rot = [-0.0359831892284 0.714725487846 -0.696155194693 0.0569265493357 ];

tf_left_pos = [ 0.331634378576   0.146365095988   0.426270038515];
tf_left_rot = [0.000752197137837 -0.679549694031 0.73354471229 -0.0111266622255 ];

tf_head_pos = [0.231875423303  -0.0981450535605  1.3859];
tf_head_rot = [-0.497051231854   0.497053100813   -0.502927806319   0.502933305236];

calc_right_pos = [0.4236   -0.1958    0.4623];
calc_right_rot = [0.0359 -0.7078    0.7035   -0.0532   ];

calc_left_pos = [0.3457    0.1468    0.4228];
calc_left_rot = [0.0068 -0.6768    0.7361    0.0081   ];

calc_head_pos = [0.2360   -0.1068    1.3660];
calc_head_rot = [0.5139  -0.4985    0.4915   -0.4958    ];


right_off_rot = quatmultiply( calc_right_rot, quatinv( tf_right_rot ) );
left_off_rot  = quatmultiply( calc_left_rot , quatinv( tf_left_rot  ) );
head_off_rot  = quatmultiply( calc_head_rot , quatinv( tf_head_rot  ) );

right_off_pos = calc_right_pos - tf_right_pos;
left_off_pos  = calc_left_pos  - tf_left_pos;
head_off_pos  = calc_head_pos  - tf_head_pos;

rot = [-pi/2 0 -pi/2];
rotmZYX = eul2rotm(rot);

right_off_pos = right_off_pos * rotmZYX;
left_off_pos  = left_off_pos  * rotmZYX;
head_off_pos  = head_off_pos  * rotmZYX;

tmp_right = right_off_rot(2:end) * rotmZYX;
tmp_left = left_off_rot(2:end)   * rotmZYX;
tmp_head = head_off_rot(2:end)   * rotmZYX;

right_off_rot = [tmp_right right_off_rot(1,1)];
left_off_rot  = [tmp_left  left_off_rot(1,1)];
head_off_rot  = [tmp_head  head_off_rot(1,1)];