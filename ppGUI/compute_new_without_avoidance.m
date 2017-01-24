function pointnew = compute_new_without_avoidance(pointold, angleold, pointall,Po,a,ob_vel)

pointold_remote(1) = pointold(1) + 3*cos(angleold);
pointold_remote(2) = pointold(2) + 3*sin(angleold);

Angle_obstaclePathgoal = compute_angles_(pointold,pointall);
[pFatx,pFaty] = compute_attract_(pointold,pointold_remote,angleold);
[pFrerxx,pFreryy,pFataxx,pFatayy]=compute_repulsion_(pointold,pointold_remote,pointall,angleold,Angle_obstaclePathgoal,Po,a);
pFsumxj=pFatx+pFrerxx+pFataxx;
pFsumyj=pFaty+pFreryy+pFatayy;
Position_angle_forcegoal = compute_angles_([0 0],[pFsumxj,pFsumyj]);
angleold=Position_angle_forcegoal;
pointnew(1) = pointold(1) + ob_vel*cos(angleold);
pointnew(2) = pointold(2) + ob_vel*sin(angleold);
