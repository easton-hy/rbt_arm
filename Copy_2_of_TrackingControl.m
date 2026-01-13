clc
clear
close all
%%
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
%%
q= 3.14* [1, 1, 1, 1, 1, 1]';
q_dot= 10*[1, 1, 1, 1, 1, 1]';
%%
T=0:0.01:15;
dt=0.02;
X=[];
kp=10; kd=2.0;

%%
for i=1:length(T)
    t=T(i);
    qd = zeros(6,1);
    qd_dot =zeros(6,1);
    qd_ddot = zeros(6,1);

    X=[X,[q;qd;q_dot]];

    V=qd_ddot-kp*(q-qd)-kd*(q_dot-qd_dot);
    %% control
    D=1.0*inertiaMatrix(q);
    g=1.0*gravityVector(q);
    cor=1.0 *cor_centriTerms(q,q_dot);
    U=D*V+g+cor;

    %% dynamcis 
    q_ddot = inertiaMatrix(q)\(U -gravityVector(q)-cor_centriTerms(q,q_dot));
    %q_ddot = inertiaMatrix(q)\(U -gravityVector(q));
    q_dot=q_ddot*dt+q_dot;
    q=q_dot*dt+q;

    %X=[X,[q;qd;q_dot]];
end

%%
figure
subplot(6,1,1)
plot(X(1,:))
hold on
plot(X(7,:),'--')
ylabel('$q_1$ (rad)')
set(gca,'xticklabel',[])


subplot(6,1,2)
plot(X(2,:))
hold on
plot(X(8,:),'--')
ylabel('$q_2$ (rad)')
set(gca,'xticklabel',[])

subplot(6,1,3)
plot(X(3,:))
hold on
plot(X(9,:),'--')
ylabel('$q_3$ (rad)')
set(gca,'xticklabel',[])

subplot(6,1,4)
plot(X(4,:))
hold on
plot(X(10,:),'--')
ylabel('$q_4$ (rad)')
set(gca,'xticklabel',[])

subplot(6,1,5)
plot(X(5,:))
hold on
plot(X(11,:),'--')
ylabel('$q_5$ (rad)')
set(gca,'xticklabel',[])

subplot(6,1,6)
plot(X(6,:))
hold on
plot(X(12,:),'--')
ylabel('$q_6$ (rad)')

%%
figure
subplot(6,1,1)
plot(X(1+12,:))
hold on
plot(X(7,:),'--')
ylabel('$dq_1$ (rad)')
set(gca,'xticklabel',[])


subplot(6,1,2)
plot(X(2+12,:))
hold on
plot(X(8,:),'--')
ylabel('$dq_2$ (rad)')
set(gca,'xticklabel',[])

subplot(6,1,3)
plot(X(3+12,:))
hold on
plot(X(9,:),'--')
ylabel('$dq_3$ (rad)')
set(gca,'xticklabel',[])

subplot(6,1,4)
plot(X(4+12,:))
hold on
plot(X(10,:),'--')
ylabel('$dq_4$ (rad)')
set(gca,'xticklabel',[])

subplot(6,1,5)
plot(X(5+12,:))
hold on
plot(X(11,:),'--')
ylabel('$dq_5$ (rad)')
set(gca,'xticklabel',[])

subplot(6,1,6)
plot(X(6+12,:))
hold on
plot(X(12,:),'--')
ylabel('$dq_6$ (rad)')
