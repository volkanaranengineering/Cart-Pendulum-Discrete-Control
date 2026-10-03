function xy=cart_pendulum_xy(z,l)
% Rows: [cart_x cart_y pendulum_COM_x pendulum_COM_y], metres.
% Original model's positive theta is counterclockwise from vertical.
if nargin<2,l=.3;end
xy=[z(:,1),zeros(size(z,1),1),z(:,1)-l*sin(z(:,3)),l*cos(z(:,3))];
end
