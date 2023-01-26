function ph = plot_sys_z(obj, T, subX, subU, subSys, ref)

    % Create axes for two column figure
    nxu = size(subX, 1) + size(subU, 1);

    targetSize = [1120, 420];
    position = obj.get_position_on_screen(targetSize);
    ph.fig = figure('Position', position);

    nrow = nxu;
    ncol = 1;

    ax = gobjects(nxu, 1);

    % Map inputs/states to subplot axes
    ux_id = 1:ncol:ncol*nxu;

    % Create subplot axes
    for ixu = 1:nxu
        ax(ux_id(ixu)) = subplot(nrow, ncol, ux_id(ixu));
        ax_ = ax(ux_id(ixu));
        hold(ax_, 'on');
        grid(ax_, 'on');
        axis(ax_, 'tight');
    end, clear ixu ax_

    % Write title and time
    title(ax(1), subSys.Name);
    xlabel(ax(max(ux_id)), 'Time [s]');

    % Create full data
    if size(subX, 1) < 13
        idx = subSys.UserData.idx;
    end
    if size(subU, 1) < 4
        idu = subSys.UserData.idu;
    end

    % Plot data
    bx = [obj.lbx(:), obj.ubx(:)];
    sub_bx = bx(idx,:);

    bu = [obj.lbu(:), obj.ubu(:)];
    sub_bu = bu(idu,:);

    % Create ref trajectory from ref argument
    subX_ref = nan(size(subX));
    subX_ref(subSys.UserData.idx == subSys.UserData.idy,:) = ref;

    obj.plot_into_axes(ax, ux_id, subSys, T, subX, subU, sub_bx, sub_bu, subX_ref);

end