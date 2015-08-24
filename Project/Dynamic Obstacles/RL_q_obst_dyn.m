side_length = 500;
m_max = 10;
n_max = 10;

state_q_values = zeros(4, 2*m_max-1, 2*n_max-1, 2, 2, 2, 2);

trajectories_max = 100000;
step = 1;
step_max = 5000;
next_rel = [1 0; 0 1; 0 -1; -1 0];

gamma = 0.98;
    
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
   
    while(1)
        obst_next = randi(2,1,4);
        if(length(obst_next(obst_next(:,:) == 2)) < 2)
            break;
        end
    end
    
    while( (m_rel ~= 0 || n_rel ~= 0) && (step <= step_max) )
        obst = obst_next;
        [max_cur_act, max_cur_index] = max(state_q_values(:,(m_rel + m_max), ...
                                            (n_rel + n_max), obst(1), obst(2), obst(3), obst(4)));
        m_change = next_rel(max_cur_index, 1);
        n_change = next_rel(max_cur_index, 2);
        m_rel_next = m_rel + m_change;
        n_rel_next = n_rel + n_change;
        
        if((abs(m_rel_next) > (m_max-1)) || (abs(n_rel_next) > (n_max-1)) || obst(max_cur_index) == 2)
            state_q_values(max_cur_index,(m_rel + m_max),(n_rel + n_max),obst(1),obst(2),obst(3),obst(4)) = -Inf;
        else
            while(1)
                obst_next = randi(2,1,4);
                if(length(obst_next(obst_next(:,:) == 2)) < 2)
                    break;
                end
            end
            max_next_act = max(state_q_values(:,(m_rel_next + m_max), ...
                                (n_rel_next + n_max), obst_next(1), obst_next(2), obst_next(3), obst_next(4)));
            dist_line = (abs(m_rel*n_change - n_rel*m_change)/(m_rel^2 + n_rel^2)^0.5);
            dist_goal = ((m_rel^2 + n_rel^2)^0.5) - abs(m_rel*m_change + n_rel*n_change)/((m_rel^2 + n_rel^2)^0.5);
            reward = -( ( 1 + dist_line)*( 1 + dist_goal) );
            state_q_values(max_cur_index,(m_rel + m_max), ...
                            (n_rel + n_max), obst(1), obst(2), obst(3), obst(4)) = reward + gamma * max_next_act;
            
            m_rel = m_rel_next;
            n_rel = n_rel_next;
            step = step + 1;
        end
    end
    step;
end

[argmax_act,pol] = max(state_q_values);
state_values = reshape(argmax_act, 2*m_max-1, 2*n_max-1, 2, 2, 2, 2);
state_policy = reshape(pol, 2*m_max-1, 2*n_max-1, 2, 2, 2, 2);