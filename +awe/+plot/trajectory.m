function trajectory(p,v,R,ld)
hold on;grid on;
s = 10;
nav2view = [1,0,0;
            0,-1,0;
            0,0,-1];
                   
for k=1:size(p,2)
  p_view = nav2view * p(:,k);
  v_view = nav2view * v(:,k);
  R_view = nav2view * reshape(R(:,k),3,3) * nav2view';
  
  % Plot reference orientation --------------------------------------------------
  ex = s*R_view*[1;0;0];
  line([p_view(1),p_view(1)+ex(1)],...
       [p_view(2),p_view(2)+ex(2)],...
       [p_view(3),p_view(3)+ex(3)],'LineWidth',2,'Color','r','LineStyle','-');

  ey = s*R_view*[0;1;0];
  line([p_view(1),p_view(1)+ey(1)],...
       [p_view(2),p_view(2)+ey(2)],...
       [p_view(3),p_view(3)+ey(3)],'LineWidth',2,'Color','g','LineStyle','-');

  ez = s*R_view*[0;0;1];
  line([p_view(1),p_view(1)+ez(1)],...
       [p_view(2),p_view(2)+ez(2)],...
       [p_view(3),p_view(3)+ez(3)],'LineWidth',2,'Color','b','LineStyle','-');
  
  % Plot velocity direction -----------------------------------------
  ev = s*v_view/norm(v_view);
  line([p_view(1),p_view(1)+ev(1)],...
       [p_view(2),p_view(2)+ev(2)],...
       [p_view(3),p_view(3)+ev(3)],'LineWidth',2,'Color','k','LineStyle',':');
 
  % Plot cable --------------------------------------------------------
  if ld(k) >= 0
    line([0,p_view(1)],[0,p_view(2)],[0,p_view(3)],'LineWidth',0.8,'Color','g','LineStyle',':');
  else
    line([0,p_view(1)],[0,p_view(2)],[0,p_view(3)],'LineWidth',0.8,'Color','r','LineStyle',':');
  end
end
legend({'forward', 'wing', 'down', 'heading'})