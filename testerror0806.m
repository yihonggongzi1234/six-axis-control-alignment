r_lower = 1078.55;      % 下平台半径 R
r_up   = 604.85;    % 上平台半径 r
d_lower = 300;
d_up = 250;
stable_height = 1800;    % 动平台台体高度
upper_radis = 900;       % 动平台旋转半径
sta2bot = 213;           % 底面到下铰点距离
upper2pt = 138;          % 动平台到上铰点距离
bt2gd = 60;              % 底面到地面距离

R = r_lower;                    % 下平台半径 R
r = r_up;                 % 上平台半径 r
D = (d_lower / 2.0);            % 下平台铰点到中心的距离
d = (d_up / 2.0);         % 上平台铰点到中心的距离
Base_H = (sta2bot + bt2gd);       % 下铰点中心距地面高度
Center_H = (stable_height + upper_radis);    % 回转中心距地面高度
Upper_Radis = upper_radis;             % 上平台回转半径
UpperJ_Radis = (upper_radis + upper2pt); % 上铰点中心回转半径

L = [1542.9824,1781.2283,1608.9531,1459.9758,1847.3476,1754.3986];

[Error_vec] = showerroraboutbar(R,r,D,d,Base_H,Center_H,UpperJ_Radis,L);
