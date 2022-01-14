// ページの読み込みが完了したらコールバック関数が呼ばれる
// ※コールバック: 第2引数の無名関数(=関数名が省略された関数)
window.addEventListener('load', () => {
  const canvas = document.querySelector('#draw-area');
  // contextを使ってcanvasに絵を書いていく
  const context = canvas.getContext('2d');
  const genarateButton = document.querySelector('#generate-button');

  // 直前のマウスのcanvas上のx座標とy座標を記録する
  const lastPosition = { x: null, y: null };
  vertical_points = [{x: 0.0, y: 0.0}];

  const prev_scripts = `-- command a Copter to takeoff to 5m and fly a vertical circle in the clockwise direction
--
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be armed and RC6 input > 1800 and then:
--    a) switches to Guided mode
--    b) takeoff to 5m
--    c) flies a vertical circle using the velocity controller
--    d) switches to RTL mode

local takeoff_alt_above_home = 5
local copter_guided_mode_num = 4
local copter_rtl_mode_num = 6
local stage = 0
local circle_angle = 0
local circle_angle_increment = 1    -- increment the target angle by 1 deg every 0.1 sec (i.e. 10deg/sec)
local circle_speed = 1              -- velocity is always 1m/s
local yaw_cos = 0                   -- cosine of yaw at takeoff
local yaw_sin = 0                   -- sine of yaw at takeoff
local count = 0
local count_end = 0
local index = 0
`;

  const after_scripts = `

-- the main update function that uses the takeoff and velocity controllers to fly a rough square pattern
function update()
  if not arming:is_armed() then -- reset state when disarmed
    gcs:send_text(0, "disarmed")
    stage = 0
    circle_angle = 0
    count = 0
    count_end = 0
    index = 0
  else
    pwm6 = rc:get_pwm(6)
    if pwm6 and pwm6 > 1800 then    -- check if RC6 input has moved high
      if (stage == 0) then          -- change to guided mode
        if (vehicle:set_mode(copter_guided_mode_num)) then  -- change to Guided mode
          local yaw_rad = ahrs:get_yaw()
          yaw_cos = math.cos(yaw_rad)
          yaw_sin = math.sin(yaw_rad)
          stage = stage + 1
        end
      elseif (stage == 1) then      -- Stage1: takeoff
        if (vehicle:start_takeoff(takeoff_alt_above_home)) then
          stage = stage + 1
        end
      elseif (stage == 2) then      -- Stage2: check if vehicle has reached target altitude
        local home = ahrs:get_home()
        local curr_loc = ahrs:get_position()
        if home and curr_loc then
          local vec_from_home = home:get_distance_NED(curr_loc)
          gcs:send_text(0, "alt above home: " .. tostring(math.floor(-vec_from_home:z())) .. " " .. tostring(takeoff_alt_above_home) .. " " .. tostring(vec_from_home:z()) .. " " .. tostring(math.abs(takeoff_alt_above_home + vec_from_home:z())))
          -- if (math.abs(takeoff_alt_above_home + vec_from_home:z()) < 1) then
          if (math.abs(vec_from_home:z()) >= takeoff_alt_above_home) then
            stage = stage + 1
            bottom_left_loc = curr_loc          -- record location when starting square
          end
        end
      elseif (stage == 3 ) then   -- Stage3: fly a vertical circle

        -- calculate velocity vector
        -- circle_angle = circle_angle + circle_angle_increment
        -- if (circle_angle >= 360) then
        --   stage = stage + 1
        -- end
        if (index + 1 >= #vertical_points) then
          stage = stage + 1
        else
          if (count == 0) then
            local x = vertical_points[index+2].x - vertical_points[index+1].x
            local y = vertical_points[index+2].y - vertical_points[index+1].y
            local rad
            local deg
            if (x > 0) then
              rad = math.atan(y/x)
              deg = math.deg(rad)
              deg = 180 - deg
            elseif (x < 0) then
              rad = math.atan(y/x)
              deg = math.deg(rad)
              if (y > 0) then
                deg = 0 - deg
              elseif (y < 0) then
                deg = 360 - deg
              else
                deg = 0
              end
            else
              if (y > 0) then
                deg = 90
              elseif (y < 0) then
                deg = 270
              else
                deg = 0 -- unexpected
              end
              rad = math.rad(deg)
            end
            circle_angle = deg
            count_end = math.sqrt(x*x + y*y) * 10
          end
          if (count < count_end) then
            gcs:send_text(0, "stage 3: " .. tostring(circle_angle) .. " " .. tostring(count) .. " " .. tostring(index))
            local target_vel = Vector3f()
            local vel_xy = math.cos(math.rad(circle_angle)) * circle_speed
            target_vel:x(yaw_sin * vel_xy)
            target_vel:y(yaw_cos * -vel_xy)
            target_vel:z(-math.sin(math.rad(circle_angle)) * circle_speed)

            -- send velocity request
            if not (vehicle:set_target_velocity_NED(target_vel)) then
              gcs:send_text(0, "failed to execute velocity command")
            end
            count = count + 1
          else
            index = index + 1
            count = 0
          end
        end
      elseif (stage == 4) then  -- Stage4: change to RTL mode
        vehicle:set_mode(copter_rtl_mode_num)
        stage = stage + 1
        gcs:send_text(0, "finished square, switching to RTL")
      end
    end
  end

  return update, 100
end

return update()
`;

  // マウスがドラッグされているか(クリックされたままか)判断するためのフラグ
  let isDrag = false;

  // 絵を書く
  function draw(x, y) {
    // マウスがドラッグされていなかったら処理を中断する。
    // ドラッグしながらしか絵を書くことが出来ない。
    if(!isDrag) {
      return;
    }

    // 「context.beginPath()」と「context.closePath()」を都度draw関数内で実行するよりも、
    // 線の描き始め(dragStart関数)と線の描き終わり(dragEnd)で1回ずつ読んだほうがより綺麗に線画書ける

    // 線の状態を定義する
    // MDN CanvasRenderingContext2D: https://developer.mozilla.org/en-US/docs/Web/API/CanvasRenderingContext2D/lineJoin
    context.lineCap = 'round'; // 丸みを帯びた線にする
    context.lineJoin = 'round'; // 丸みを帯びた線にする
    context.lineWidth = 5; // 線の太さ
    context.strokeStyle = 'black'; // 線の色

    // 書き始めは lastPosition.x, lastPosition.y の値はnullとなっているため、
    // クリックしたところを開始点としている。
    // この関数(draw関数内)の最後の2行で lastPosition.xとlastPosition.yに
    // 現在のx, y座標を記録することで、次にマウスを動かした時に、
    // 前回の位置から現在のマウスの位置まで線を引くようになる。
    if (lastPosition.x === null || lastPosition.y === null) {
      // ドラッグ開始時の線の開始位置
      context.moveTo(x, y);
    } else {
      // ドラッグ中の線の開始位置
      context.moveTo(lastPosition.x, lastPosition.y);
    }
    // context.moveToで設定した位置から、context.lineToで設定した位置までの線を引く。
    // - 開始時はmoveToとlineToの値が同じであるためただの点となる。
    // - ドラッグ中はlastPosition変数で前回のマウス位置を記録しているため、
    //   前回の位置から現在の位置までの線(点のつながり)となる
    context.lineTo(x, y);

    // context.moveTo, context.lineToの値を元に実際に線を引く
    context.stroke();

    // 現在のマウス位置を記録して、次回線を書くときの開始点に使う
    lastPosition.x = x;
    lastPosition.y = y;
    x = (x - 250)/50;
    y = (500 - y)/50;
    if (Math.abs(vertical_points[vertical_points.length - 1].x - x) > 0.1 || Math.abs(vertical_points[vertical_points.length - 1].y - y) >= 0.1) {
      vertical_points.push({x: x, y: y});
    }
    if (vertical_points.length >= 2) {
      genarateButton.disabled = false;
    }
  }

  // canvas上に書いた絵を全部消す
  function clear() {
    context.clearRect(0, 0, canvas.width, canvas.height);
    vertical_points = [{x: 0.0, y: 0.0}];
    genarateButton.disabled = true;
  }

  // canvas上に書いた絵を全部書き出す
  function generate() {
    if (vertical_points.length >= 2) {
      console.log("generate!!");
      let script = "local vertical_points = {";
      for (let i = 0; i < vertical_points.length; i++) {
        if (i != 0) {
          script = script + ", ";
        }
        script = script + "{x=" + vertical_points[i].x + ", y=" + vertical_points[i].y + "}";
      }
      script = script + "}";
      let bom  = new Uint8Array([0xEF, 0xBB, 0xBF]);
      let blob = new Blob([bom, prev_scripts, script, after_scripts], {type: 'text/plain'});
      let link = document.createElement('a');
      link.href = URL.createObjectURL(blob);
      link.download = 'copter-fly-verticals.lua';
      link.click();
      console.log(prev_scripts);
      console.log(script);
      console.log(after_scripts);
    }
  }

  // マウスのドラッグを開始したらisDragのフラグをtrueにしてdraw関数内で
  // お絵かき処理が途中で止まらないようにする
  function dragStart(event) {
    // これから新しい線を書き始めることを宣言する
    // 一連の線を書く処理が終了したらdragEnd関数内のclosePathで終了を宣言する
    context.beginPath();

    isDrag = true;
  }
  // マウスのドラッグが終了したら、もしくはマウスがcanvas外に移動したら
  // isDragのフラグをfalseにしてdraw関数内でお絵かき処理が中断されるようにする
  function dragEnd(event) {
    // 線を書く処理の終了を宣言する
    context.closePath();
    isDrag = false;

    // 描画中に記録していた値をリセットする
    lastPosition.x = null;
    lastPosition.y = null;
  }

  // マウス操作やボタンクリック時のイベント処理を定義する
  function initEventHandler() {
    const clearButton = document.querySelector('#clear-button');
    clearButton.addEventListener('click', clear);
    genarateButton.addEventListener('click', generate);
    genarateButton.disabled = true;

    canvas.addEventListener('mousedown', dragStart);
    canvas.addEventListener('mouseup', dragEnd);
    canvas.addEventListener('mouseout', dragEnd);
    canvas.addEventListener('mousemove', (event) => {
      // eventの中の値を見たい場合は以下のようにconsole.log(event)で、
      // デベロッパーツールのコンソールに出力させると良い
      // console.log(event);

      draw(event.layerX, event.layerY);
    });
  }

  // イベント処理を初期化する
  initEventHandler();
});
