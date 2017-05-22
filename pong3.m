function pong

% PONG Matlab version of the eponym game
% 
%   PONG was created in the early 70's by Nolan Bushnell from Atari Inc. 
%   This code should be considered as a tribute.
%   More info on Wikipedia:
%       http://en.wikipedia.org/wiki/Pong
%       http://en.wikipedia.org/wiki/Atari
% 
%   Controls: 
%       Right player uses mouse
%       Left player uses up/down keyboard arrows
% 
%   Players can also use following keys:
%       P: game paused
%       S: switch sound on/off
%       Esc: quit game
%
        
%   Author: Jrme Briot
%   Contact: dutmatlab@yahoo.fr
%   Creation: Sep 2006
% 
%   Revision: 
%   #1: Oct 2006 - improve ball management
%                - add sound options
% 

% Get screen size
set(0,'units','pixels')
scr=get(0,'screensize');
%scr

% Create figure
figure('units','pixels','position',[.25*scr(3) .25*scr(4) .5*scr(3) .5*scr(4)],...
    'color','k',...
    'toolbar','none',...
    'menubar','none',...
    'numbertitle','off',...
    'name','PONG - dutmatlab@yahoo.fr - Sept 2006',...
    'doublebuffer','on',...
    'pointer','custom',...
    'pointershapecdata',repmat(nan,16,16),...
    'closerequestfcn',[])

% Separate the next line because of bug in R12.1 
set(gcf,'resize','off')

% Create axes  
axes('units','normalized','position',[0 0 1 1],...
    'xtick',[],'ytick',[],'color','k','xlim',[0 1],'ylim',[0 1])

% Create animated splash screen
t(1)=text(.35,.75,'P','fontname','courier','fontsize',60,'color','k','hor','center','fontweight','bold');
t(2)=text(.45,.75,'O','fontname','courier','fontsize',60,'color','k','hor','center','fontweight','bold');
t(3)=text(.55,.75,'N','fontname','courier','fontsize',60,'color','k','hor','center','fontweight','bold');
t(4)=text(.65,.75,'G','fontname','courier','fontsize',60,'color','k','hor','center','fontweight','bold');
t(5)=text(.50,.55,'A tribute to the eponym game by ATARI Inc.','fontname','courier','fontsize',20,'color','k','hor','center','fontweight','bold');
t(6)=text(.50,.35,'by Jrme Briot','fontname','courier','fontsize',35,'color','k','hor','center','fontweight','bold');
t(7)=text(.50,.25,'dutmatlab@yahoo.fr','fontname','courier','fontsize',20,'color','k','hor','center','fontweight','bold');
t(8)=text(.50,.15,'(Hit a key)','fontname','courier','fontsize',20,'color','k','hor','center','fontweight','bold');

map=gray(128);

for m=1:length(t)
    
    for n=1:size(map,1)
        
        set(t(m),'color',map(n,:))
        drawnow
        
    end
    
end

% Wait for player to hit a key to remove the splash screen 
pause

set(gcf,'closerequestfcn','closereq')
delete(t)

hold on

% Draw line at the middle
plot([.5 .5],[.05 .95],'r:','color',[.8 .8 .8],'linewidth',6)

% Draw upper line
patch([0 1 1 0],[.95 .95 .98 .98],[0 0 0 0],'facecolor','w','edgecolor','w',...
    'handlevisibility','off')

% Draw lower line
patch([0 1 1 0],[.02 .02 .05 .05],[0 0 0 0],'facecolor','w','edgecolor','w',...
    'handlevisibility','off')

% Draw left player racket
patch([.05 .07 .07 .05],[.455 .455 .545 .545],[0 0 0 0],'facecolor','b','edgecolor','k','tag','left_player')


% Draw right player racket
patch([.95 .93 .93 .95],[.455 .455 .545 .545],[0 0 0 0],'facecolor','r','edgecolor','k','tag','right_player')

% Draw the ball
h_ball=patch([.492 .508 .508 .492],[.492 .492 .508 .508],[-1 -1 -1 -1],'facecolor','w','edgecolor','w','tag','ball');

% Add text for score
text(.425 , .86,'0','fontname','courier','fontsize',60,'color','w','hor','center','tag','score_left')
text(.575 , .86,'0','fontname','courier','fontsize',60,'color','w','hor','center','tag','score_right');

% Add pause text 
t_pause=text(.5,.5,'Game paused','fontname','courier','fontsize',60,'color','w','hor','center','fontweight','bold','visible','off');

% Set player controls
% Left player -> KeyPressFcn
% Right player -> WindowButtonMotionFcn
set(gcf,'keypressfcn',@kpfcn,'windowbuttonmotionfcn',@wbmfcn)

% Load the sound for the rebonds
[pong_sounds(1).Y,pong_sounds(1).FS]=wavread('pong.wav');
setappdata(gcf,'pong_sounds',pong_sounds)

% Set random direction for the ball
rand('state',sum(100*clock));

a=rand;

xdir=[-1 1];
x_sign=randperm(2);
xdir=xdir(x_sign(1));

ydir=[-1 1];
y_sign=randperm(2);
ydir=ydir(y_sign(1));

% Wait for player to hit a key to start the game
pause
set(gcf,'currentchar','a')

% Set sound parameter to on
sound_level=1;

% Initialize variables
flag=1;
incr=.025;
map=gray(256);
tempo_init=.1;
elapsed_time=0;
prev_left = [.06, .5];
prev_right = [.94, .5];

% Game action
while flag
    
    % Increase the speed of the ball during the game
    tempo=tempo_init/(floor(elapsed_time/3)+1);
    
    % Check key pressed 
    if get(gcf,'currentchar')==112 | get(gcf,'currentchar')==80 % P => switch pause on/off
        
        set(gcf,'currentcharacter','a');
        
        set(t_pause,'visible','on')
        n=1;
        
        % Make the pause sentence to blink
        while get(gcf,'currentchar')~=112 & get(gcf,'currentchar')~=80 

           set(t_pause,'color',map(n,:));
           drawnow
           
           n=n+1;
           
           if n==257;
               
                map=map(end:-1:1,:);
                n=1;
                
           end   
            
        end
        
        set(t_pause,'visible','off')
        set(gcf,'currentchar','a')
        
    elseif get(gcf,'currentchar')==27 % Esc => quit game

        flag=0;
        cla
        text(.5,.5,'Bye !','fontname','courier','fontsize',60,'color','w','hor','center','fontweight','bold')
        pause(.5)
        break
        
    elseif get(gcf,'currentchar')==115 % S => switch sound on/off

        set(gcf,'currentchar','a')
        sound_level=~sound_level;
        
    end
    
    % Move the ball
    x=get(h_ball,'xdata')+xdir*0.025*a;
    y=get(h_ball,'ydata')+ydir*a*0.025;
    
    % Check the position of the ball

    [xdir,ydir,point, prev_left, prev_right]=checkpos(x,y,xdir,ydir,sound_level, prev_left, prev_right);

    if xdir==0 
       
        x=[.492 .508 .508 .492];
        y=[.492 .492 .508 .508];
        xdir=-1;
        a=rand;
        
    end
    
    if ydir==0
        
        x=[.492 .508 .508 .492];
        y=[.492 .492 .508 .508];
        xdir=1;
        a=rand;
        ydir=[-1 1];
        y_sign=randperm(2);
        ydir=ydir(y_sign(1));
        
    end        
    
    set(h_ball,'xdata',x,'ydata',y)
    
    % Reset ball speed in case of point
    if point
        
        elapsed_time=0;
        
    else
        
        elapsed_time=elapsed_time+tempo;
        
    end
    
    pause(tempo)
    drawnow
    
end

close

% End of the PONG function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function kpfcn(obj,event)

% KPFCN Control the racket of left player
%
%   KPFCN moves the racket of left player using keyboard
%

ck = get(obj,'currentkey');
h_left = findobj('tag','left_player');

%idx=strcmp(ck,{'uparrow' 'downarrow'});

idx = (ck == [101, 100, 115, 102]); % [e, d, s, f]

yd = get(h_left,'ydata'); 
xd = get(h_left, 'xdata');

yd = yd + .02*(idx(1) & yd(3) < .94) -.02 *(idx(2) & yd(1)>.06);
xd = xd + .02*(idx(4) & xd(3) < .5)  -.02 *(idx(3) & xd(1) > .05);

set(h_left,'ydata',yd, 'xdata', xd)
drawnow

% End of the KPFCN function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function wbmfcn(obj,event)

% WBMFCN    Control the racket of right player
%
%   KPFCN moves the racket of right player using mouse
%
%

h_right=findobj('tag','right_player');

cp=get(gca,'currentpoint');

cpx =(cp(1,1) +[-.01 -.01 .01 .01]).*(cp(1,1) > .5 & cp(1,1) < .94) + (.5+[-.01 -.01 .01 .01])*(cp(1,1) < .5)+(.94+[-.01 -.01 .01 .01])*(cp(1,1)>.94);
cpx = reshape(cpx, 4,1);
cpx(2,1) = cpx(4,1);
cpx(4,1) = cpx(1,1);
cpy =(cp(1,2)+[-.045 -.045 .045 .045]).*(cp(1,2)>.105 & cp(1,2)<.895)+(.105+[-.045 -.045 .045 .045])*(cp(1,2)<.105)+(.895+[-.045 -.045 .045 .045])*(cp(1,2)>.895);
cpy = reshape(cpy, 4, 1);
set(h_right,'ydata',cpy, 'xdata', cpx);

% End of the WBMFCN function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [new_xdir,new_ydir,point, prev_left, prev_right]=checkpos(x,y,xdir,ydir,sound_level, prev_left, prev_right)
% CHECKPOS  Check the position of the ball
%

% Find rackets object
h_right=findobj('tag','right_player');
h_left=findobj('tag','left_player');
% Get rackets position
y_right=get(h_right,'ydata');
y_left=get(h_left,'ydata');

% Set sound feature
pong_sounds=getappdata(gcf,'pong_sounds');

if ~sound_level
    
    pong_sounds(1).Y=0;
    
end

% Check if the ball hits any racket
keeper_right=any(inpolygon(x+.008,y+.008,get(h_right,'xdata'),get(h_right,'ydata')));
keeper_left=any(inpolygon(x-.008,y-.008,get(h_left,'xdata'),get(h_left,'ydata')));

% The ball hits the upper/lower limit
if y(3)>.942 | y(1)<.058
    
    wavplay(pong_sounds(1).Y,pong_sounds(1).FS)
    
    new_ydir=-ydir;
    new_xdir=xdir;
    point=0;
    
% The ball hits the racket of the right player

elseif keeper_right

    [prev_left, prev_right, strike_left, strike_right] = collision(prev_left, prev_right, h_left, h_right);
    wavplay(pong_sounds(1).Y,pong_sounds(1).FS)
    
    if strike_right(1) < 0
        new_xdir = xdir + (1 + strike_right(1)) * xdir;
    else%if strike_right(1) > 0
        new_xdir = -xdir + (1 + strike_right(1)) * -xdir;
    end
    %new_xdir= -xdir;
    new_ydir=ydir;  
    point=0;

% The ball hits the racket of the left player
elseif keeper_left

    [prev_left, prev_right, strike_left, strike_right] = collision(prev_left, prev_right, h_left, h_right);
    wavplay(pong_sounds(1).Y,pong_sounds(1).FS)
    
    new_xdir=-xdir;
    new_ydir=ydir;   
    point=0;
 
% One point for the right player
elseif (x+.008)>.93 
    
    sc_left=findobj('type','text','tag','score_left');
    score=get(sc_left,'string')-48+1;
    set(sc_left,'string',score);
    new_xdir=0;
    new_ydir=1;
    point=1;
    
% One point for the left player
elseif (x+.008)<.0.07 
    
    sc_right=findobj('type','text','tag','score_right'); 
    score=get(sc_right,'string')-48+1;
    set(sc_right,'string',score)
    new_xdir=1;
    new_ydir=0;
    point=1;
    
% The ball keep moving
else new_xdir=xdir;
     new_ydir=ydir;
     point=0;
     
end

% End of the CHECKPOS function

function [prev_left, prev_right, strike_left, strike_right] = collision(prev_left, prev_right, h_left, h_right)
    xleft = get(h_left, 'xdata');
    yleft = get(h_left, 'ydata');
    xright = get(h_right, 'xdata');
    yright = get(h_right, 'ydata');
    cur_left = [xleft(1,1) + .01, yleft(1,1) + .045]
    prev_left
    cur_right = [(xright(1,1) + .01), (yright(1,1) + .045)]
    prev_right
    change_left = cur_left - prev_left
    change_right = cur_right - prev_right
    
    change_time = .1
    
    strike_left = change_left ./ change_time
    strike_right = change_right ./ change_time
    prev_left = cur_left;
    prev_right = cur_right;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
