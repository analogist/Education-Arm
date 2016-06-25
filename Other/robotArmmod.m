/*
Copyright (c) 2015-2016, James Wu
Arm plotting modified from https://www.mathworks.com/matlabcentral/fileexchange/26660-robotarm

Copyright (c) 2010, Dmitry Savransky
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the distribution

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

function robotArmmod

h = figure;
hold on;
l1 = 12;
l2 = 12;
th1 = -pi/4;
th2 = pi/2;

xlim(gca, [-l2-2 (l1+l2)+2]);
ylim(gca, [-l2-2 (l1+l2)+2]);

plot(0,0,'o', 'MarkerSize', 60);
pos1 = [0 + cos(th1)*l1; 0 + sin(th1)*l1];
pos2 = [pos1(1) + cos(th2+th1)*l1; pos1(2) + sin(th2+th1)*l1];
joint1 = plot([0; pos1(1)], [0; pos1(2)], 'LineWidth', 10, 'Color', 'blue');
joint2 = plot([pos1(1); pos2(1)], [pos1(2); pos2(2)], 'LineWidth', 6, 'Color', 'red');

stoppressed = 0;
while stoppressed ~= 1;
    qpos = pos2;
    set(gcf,'CurrentCharacter','@'); % set to a dummy character
    pressedkey=get(gcf,'CurrentCharacter');
    while pressedkey == '@'
        pressedkey=get(gcf,'CurrentCharacter');
        pause(0.01);
    end
    figure(h);
    set(gcf,'CurrentCharacter','@');
    switch(pressedkey)
        case 'q'
            if(th1+pi/40 <= pi/2)
                th1 = th1+pi/180;
%                 th2 = th2+pi/100;
            end
        case 'a'
            if(th1-pi/40 >= -pi/2)
                th1 = th1-pi/180;
%                 th2 = th2-pi/100;
            end
        case 'w'
            if(th2+pi/20 <= 2*pi/3)
                th2 = th2+pi/180;
            end
        case 's'
            if(th2-pi/20 >= -2*pi/3)
                th2 = th2-pi/180;
            end
        case 'i'
            qpos(2) = qpos(2)+0.2;
        case 'k'
            qpos(2) = qpos(2)-0.2;
        case 'j'
            qpos(1) = qpos(1)-0.2;
        case 'l'
            qpos(1) = qpos(1)+0.2;
        case '='
            stoppressed = 1;
    end
    
    if (any(qpos ~= pos2))
%         qpos
%         c2 = (qpos(1).^2 + qpos(2).^2);
%         q1 = atan2(qpos(2),qpos(1));
%         q2 = acos(l1.^2-l2.^2+c2)/2/l1/sqrt(c2);
%         th1 = q1+q2;
%         th2 = acos((l1.^2+l2.^2-c2)/2/l1/l2);

        c2 = (qpos(1)^2 + qpos(2)^2 - l1^2 - l2^2)/(2*l1*l2);
        s2 = sqrt(1 - c2.^2);
        th2 = atan2(s2, c2); % theta2 is deduced

        k1 = l1 + l2.*c2;
        k2 = l2*s2;
        th1 = atan2(qpos(2), qpos(1)) - atan2(k2, k1); % theta1 is deduced
    end
    
    ppos = pos2;
    pos1 = [0 + cos(th1)*l1; 0 + sin(th1)*l1];
    pos2 = [pos1(1) + cos(th2+th1)*l1; pos1(2) + sin(th2+th1)*l1];
    set(joint1, 'XData', [0; pos1(1)]);
    set(joint1, 'YData', [0; pos1(2)]);
    set(joint2, 'XData', [pos1(1); pos2(1)]);
    set(joint2, 'YData', [pos1(2); pos2(2)]);
    
    plot([ppos(1) pos2(1)], [ppos(2) pos2(2)], 'k');

    [pos1 pos2 [th1; th2]]
    
    
end

close all;

end
