%load QValues.mats
side_length = 500;
m_max = 100;
n_max = 100;

%state_q_values = zeros(8, (2*(m_max-1)+1), (2*(n_max-1)+1));

trajectories_max = 5000000;
step = 1;
step_max = 50000;
next_rel = [1 1; 1 0; 1 -1; 0 1; 0 -1; -1 1; -1 0; -1 -1];

gamma = 0.98;
lam_1 = 12;
lam_2 = 1;

for trajectory = 1:trajectories_max
    step = 1;
    m_goal = randi(m_max);
    n_goal = randi(n_max);
    
    while(1)
        m_start = randi(m_max);
        n_start = randi(n_max);
    
        if((m_start ~= m_goal) || (n_start ~= n_goal))
            m_rel = m_goal - m_start;
            n_rel = n_goal - n_start;
            break;
        end
    end
    
    while( (m_rel ~= 0 || n_rel ~= 0) && (step <= step_max) )
        
        [max_cur_act, max_cur_index] = max(state_q_values(:,(m_rel + m_max), (n_rel + n_max)));
        m_change = next_rel(max_cur_index, 1);
        n_change = next_rel(max_cur_index, 2);
        m_rel_next = m_rel + m_change;
        n_rel_next = n_rel + n_change;
        
        if((abs(m_rel_next) > (m_max-1)) || (abs(n_rel_next) > (n_max-1)))
            state_q_values(max_cur_index, (m_rel + m_max), (n_rel + n_max)) = -Inf;
        else
            max_next_act = max(state_q_values(:,(m_rel_next + m_max), (n_rel_next + n_max)));
            dist_line = (abs(m_rel*n_change - n_rel*m_change)/(m_rel^2 + n_rel^2)^0.5);
            dist_goal = ((m_rel^2 + n_rel^2)^0.5) - abs(m_rel*m_change + n_rel*n_change)/((m_rel^2 + n_rel^2)^0.5);
            reward = -( ( 1 + dist_line)*( 1 + dist_goal) );
            state_q_values( max_cur_index, (m_rel + m_max), (n_rel + n_max)) = reward + gamma * max_next_act;
            
            m_rel = m_rel_next;
            n_rel = n_rel_next;
            step = step + 1
        end
    end
    step;
end

[~,I] = sort(state_q_values);
Pol8 = reshape(I(1,:,:), (2*(m_max-1)+1), (2*(n_max-1)+1));
Pol7 = reshape(I(2,:,:), (2*(m_max-1)+1), (2*(n_max-1)+1));
Pol6 = reshape(I(3,:,:), (2*(m_max-1)+1), (2*(n_max-1)+1));
Pol5 = reshape(I(4,:,:), (2*(m_max-1)+1), (2*(n_max-1)+1));
Pol4 = reshape(I(5,:,:), (2*(m_max-1)+1), (2*(n_max-1)+1));
Pol3 = reshape(I(6,:,:), (2*(m_max-1)+1), (2*(n_max-1)+1));
Pol2 = reshape(I(7,:,:), (2*(m_max-1)+1), (2*(n_max-1)+1));
Pol1 = reshape(I(8,:,:), (2*(m_max-1)+1), (2*(n_max-1)+1));

csvwrite('Pol1.csv', Pol1);
csvwrite('Pol2.csv', Pol2);
csvwrite('Pol3.csv', Pol3);
csvwrite('Pol4.csv', Pol4);
csvwrite('Pol5.csv', Pol5);
csvwrite('Pol6.csv', Pol6);
csvwrite('Pol7.csv', Pol7);
csvwrite('Pol8.csv', Pol8);

[argmax_act,pol] = max(state_q_values);
state_values = reshape(argmax_act, (2*(m_max-1)+1), (2*(n_max-1)+1));
state_policy = reshape(pol, (2*(m_max-1)+1), (2*(n_max-1)+1));