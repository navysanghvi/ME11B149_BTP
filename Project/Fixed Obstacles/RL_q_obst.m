side_length = 500;
m_max = 10;
n_max = 10;

map = zeros(m_max,n_max);
p1 = 1; p2 = 2; p3 = 3; p4 = 4; p5 = 5; p6 = 6; p7 = 7; p8 = 8; p9 = 9; p10 = 10;
map(p9:p10, p4:p5) = ones(2,2);
map(p3:p5, p3:p5) = ones(3,3);
map(p1:p2, p9:p10) = ones(2,2);
map(p4:p5, p7:p8) = ones(2,2);

state_q_values = zeros(8, m_max, n_max);

trajectories_max = 5000000;
step = 1;
step_max = 50000;
next_rel = [1 1; 1 0; 1 -1; 0 1; 0 -1; -1 1; -1 0; -1 -1];

gamma = 0.98;

m_goal = 5;
n_goal = 6;
    
for trajectory = 1:trajectories_max
    step = 1;
    
    while(1)
        m_start = randi(m_max);
        n_start = randi(n_max);
    
        if((m_start ~= m_goal) || (n_start ~= n_goal))
            if(map(m_start, n_start) == 0) 
                break;
            end
        end
    end
    
    m = m_start;
    n = n_start;
    
    while( (m ~= m_goal || n ~= n_goal) && (step <= step_max) )
        
        [max_cur_act, max_cur_index] = max(state_q_values(:,m, n));
        m_change = next_rel(max_cur_index, 1);
        n_change = next_rel(max_cur_index, 2);
        m_next = m + m_change;
        n_next = n + n_change;
        m_rel = m_goal - m;
        n_rel = n_goal - n;
        
        if((m_next > m_max) || (n_next > n_max) || (m_next <= 0) || (n_next <= 0) || map(m_next,n_next) == 1)
            state_q_values(max_cur_index, m, n) = -Inf;
        else
            max_next_act = max(state_q_values(:, m_next, n_next));
            dist_line = (abs(m_rel*n_change - n_rel*m_change)/(m_rel^2 + n_rel^2)^0.5);
            dist_goal = ((m_rel^2 + n_rel^2)^0.5) - abs(m_rel*m_change + n_rel*n_change)/((m_rel^2 + n_rel^2)^0.5);
            reward = -( ( 1 + dist_line)*( 1 + dist_goal) );
            state_q_values( max_cur_index, m, n) = reward + gamma * max_next_act;
            
            m = m_next;
            n = n_next;
            step = step + 1
        end
    end
    step
end

[argmax_act,pol] = max(state_q_values);
state_values = reshape(argmax_act, m_max, n_max);
state_policy = reshape(pol, m_max, n_max);