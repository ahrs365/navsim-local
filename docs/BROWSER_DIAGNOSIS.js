// 浏览器端诊断脚本
// 在浏览器控制台（F12 → Console）中粘贴并运行此脚本

function diagnoseTrajectory() {
    console.log('%c=== NavSim 轨迹诊断 ===', 'color: #00ff00; font-size: 16px; font-weight: bold');
    console.log('');
    
    // 1. 检查连接状态
    console.log('%c1. 连接状态', 'color: #00aaff; font-weight: bold');
    console.log('   Connected:', state.connected ? '✅' : '❌');
    console.log('   Socket:', state.socket ? '✅' : '❌');
    if (!state.connected || !state.socket) {
        console.log('%c   ⚠️  请点击"连接 WebSocket"按钮', 'color: #ff9900');
        return;
    }
    
    // 2. 检查轨迹线对象
    console.log('');
    console.log('%c2. 轨迹线对象', 'color: #00aaff; font-weight: bold');
    console.log('   Exists:', state.trajectoryLine ? '✅' : '❌');
    console.log('   Visible:', state.trajectoryLine?.visible ? '✅' : '❌');
    console.log('   Points:', state.trajectoryLine?.geometry?.drawRange?.count || 0);
    console.log('   Color:', state.trajectoryLine?.material?.color);
    
    // 3. 检查轨迹回放
    console.log('');
    console.log('%c3. 轨迹回放状态', 'color: #00aaff; font-weight: bold');
    console.log('   Active:', state.trajectoryPlayback ? '✅' : '❌');
    if (state.trajectoryPlayback) {
        console.log('   Points:', state.trajectoryPlayback.trajectory?.length || 0);
        console.log('   Start time:', state.trajectoryPlayback.startTime);
        const elapsed = (performance.now() - state.trajectoryPlayback.startTime) / 1000;
        console.log('   Elapsed:', elapsed.toFixed(2), 's');
    }
    
    // 4. 检查话题日志
    console.log('');
    console.log('%c4. 话题日志', 'color: #00aaff; font-weight: bold');
    const allTopics = Array.from(state.topicLogs.keys());
    const planTopics = allTopics.filter(t => t.includes('plan'));
    console.log('   Total topics:', allTopics.length);
    console.log('   Plan topics:', planTopics.length);
    
    if (planTopics.length > 0) {
        console.log('   Plan topics found:');
        planTopics.forEach(topic => {
            const logs = state.topicLogs.get(topic);
            console.log(`     - ${topic}: ${logs?.length || 0} messages`);
            if (logs && logs.length > 0) {
                const lastLog = logs[logs.length - 1];
                console.log('       Last message:', lastLog);
            }
        });
    } else {
        console.log('%c   ⚠️  未找到 plan 相关话题', 'color: #ff9900');
        console.log('   可能的原因：');
        console.log('   - 本地算法未启动');
        console.log('   - 本地算法未发送 plan');
        console.log('   - Topic 名称不匹配');
    }
    
    // 5. 检查最近的消息
    console.log('');
    console.log('%c5. 最近的消息', 'color: #00aaff; font-weight: bold');
    const recentTopics = state.topicOrder.slice(-10);
    console.log('   Last 10 topics:', recentTopics);
    
    // 6. 测试 topic 匹配
    console.log('');
    console.log('%c6. Topic 匹配测试', 'color: #00aaff; font-weight: bold');
    const testTopics = [
        'room/demo/plan_update',
        '/room/demo/plan_update',
        'room/demo/plan',
        '/room/demo/plan'
    ];
    
    testTopics.forEach(topic => {
        const matches = topic.endsWith('/plan_update');
        const symbol = matches ? '✅' : '❌';
        console.log(`   ${symbol} "${topic}" → endsWith('/plan_update'): ${matches}`);
    });
    
    // 7. 添加消息监听器
    console.log('');
    console.log('%c7. 添加消息监听器', 'color: #00aaff; font-weight: bold');
    
    if (window._navsimDebugListener) {
        console.log('   监听器已存在，跳过');
    } else {
        const originalOnMessage = state.socket.onmessage;
        state.socket.onmessage = (event) => {
            try {
                const parsed = JSON.parse(event.data);
                if (parsed.topic && parsed.topic.includes('plan')) {
                    console.log('%c[PLAN MESSAGE]', 'color: #ff00ff; font-weight: bold', parsed.topic);
                    console.log('   Data keys:', Object.keys(parsed.data || {}));
                    if (parsed.data?.trajectory) {
                        console.log('   Trajectory length:', parsed.data.trajectory.length);
                        console.log('   First point:', parsed.data.trajectory[0]);
                    }
                    if (parsed.data?.points) {
                        console.log('   ⚠️  使用了 points 字段（应该是 trajectory）');
                    }
                }
            } catch (e) {
                // Ignore parse errors
            }
            originalOnMessage(event);
        };
        window._navsimDebugListener = true;
        console.log('   ✅ 监听器已添加');
        console.log('   现在会打印所有包含 "plan" 的消息');
    }
    
    // 8. 检查 interpretMessage 函数
    console.log('');
    console.log('%c8. 检查消息处理函数', 'color: #00aaff; font-weight: bold');
    console.log('   interpretMessage:', typeof interpretMessage);
    console.log('   handlePlanUpdate:', typeof handlePlanUpdate);
    console.log('   updateTrajectory:', typeof updateTrajectory);
    
    // 9. 手动测试轨迹渲染
    console.log('');
    console.log('%c9. 手动测试轨迹渲染', 'color: #00aaff; font-weight: bold');
    console.log('   运行以下命令测试轨迹渲染：');
    console.log('   testTrajectoryRendering()');
    
    console.log('');
    console.log('%c=== 诊断完成 ===', 'color: #00ff00; font-size: 16px; font-weight: bold');
    console.log('');
    console.log('如果看到 plan 消息但没有轨迹，请检查：');
    console.log('1. Topic 是否以 /plan_update 结尾');
    console.log('2. Data 是否包含 trajectory 字段（不是 points）');
    console.log('3. Trajectory 中的点是否包含 yaw 字段（不是 theta）');
}

// 手动测试轨迹渲染
function testTrajectoryRendering() {
    console.log('%c=== 测试轨迹渲染 ===', 'color: #00ff00; font-weight: bold');
    
    // 创建测试轨迹
    const testTrajectory = [];
    for (let i = 0; i < 100; i++) {
        const t = i * 0.1;
        testTrajectory.push({
            x: i * 0.2,
            y: Math.sin(i * 0.1) * 2,
            yaw: i * 0.05,
            t: t
        });
    }
    
    console.log('创建测试轨迹:', testTrajectory.length, '个点');
    
    // 调用 updateTrajectory
    try {
        updateTrajectory(testTrajectory);
        console.log('✅ updateTrajectory 调用成功');
        console.log('   Trajectory visible:', state.trajectoryLine?.visible);
        console.log('   Trajectory points:', state.trajectoryLine?.geometry?.drawRange?.count);
        
        if (state.trajectoryLine?.visible) {
            console.log('%c✅ 轨迹应该可见！', 'color: #00ff00; font-weight: bold');
            console.log('如果在 3D 场景中看不到绿色曲线，可能是：');
            console.log('- 相机角度问题（尝试旋转视角）');
            console.log('- 轨迹在视野外（尝试缩放）');
            console.log('- 渲染问题（刷新页面）');
        } else {
            console.log('%c❌ 轨迹不可见', 'color: #ff0000; font-weight: bold');
        }
    } catch (e) {
        console.log('%c❌ updateTrajectory 调用失败', 'color: #ff0000; font-weight: bold');
        console.error(e);
    }
}

// 监听 handlePlanUpdate 调用
function monitorHandlePlanUpdate() {
    console.log('%c=== 监听 handlePlanUpdate ===', 'color: #00ff00; font-weight: bold');
    
    if (window._originalHandlePlanUpdate) {
        console.log('监听器已存在，跳过');
        return;
    }
    
    window._originalHandlePlanUpdate = handlePlanUpdate;
    window.handlePlanUpdate = function(data, topic) {
        console.log('%c[handlePlanUpdate CALLED]', 'color: #ff00ff; font-weight: bold');
        console.log('   Topic:', topic);
        console.log('   Data:', data);
        console.log('   Trajectory:', data?.trajectory);
        console.log('   Trajectory length:', data?.trajectory?.length);
        
        const result = window._originalHandlePlanUpdate(data, topic);
        
        console.log('   After call:');
        console.log('   Trajectory visible:', state.trajectoryLine?.visible);
        console.log('   Trajectory points:', state.trajectoryLine?.geometry?.drawRange?.count);
        console.log('   Playback active:', !!state.trajectoryPlayback);
        
        return result;
    };
    
    console.log('✅ 监听器已添加');
    console.log('现在会打印所有 handlePlanUpdate 调用');
}

// 显示帮助
function showHelp() {
    console.log('%c=== NavSim 调试帮助 ===', 'color: #00ff00; font-size: 16px; font-weight: bold');
    console.log('');
    console.log('可用函数：');
    console.log('  diagnoseTrajectory()        - 运行完整诊断');
    console.log('  testTrajectoryRendering()   - 测试轨迹渲染');
    console.log('  monitorHandlePlanUpdate()   - 监听 handlePlanUpdate 调用');
    console.log('  showHelp()                  - 显示此帮助');
    console.log('');
    console.log('快速开始：');
    console.log('  1. 运行 diagnoseTrajectory()');
    console.log('  2. 查看输出，找出问题');
    console.log('  3. 如果需要，运行 testTrajectoryRendering() 测试渲染');
    console.log('  4. 如果需要，运行 monitorHandlePlanUpdate() 监听调用');
}

// 自动运行诊断
console.log('%c欢迎使用 NavSim 调试工具！', 'color: #00ff00; font-size: 14px; font-weight: bold');
console.log('运行 showHelp() 查看可用函数');
console.log('运行 diagnoseTrajectory() 开始诊断');
console.log('');

// 导出函数到全局
window.diagnoseTrajectory = diagnoseTrajectory;
window.testTrajectoryRendering = testTrajectoryRendering;
window.monitorHandlePlanUpdate = monitorHandlePlanUpdate;
window.showHelp = showHelp;

