#pragma once

namespace navsim {
namespace plugin {

/**
 * @brief 初始化所有插件
 * 
 * 这个函数强制链接器包含所有插件的注册代码。
 * 必须在使用插件系统之前调用。
 */
void initializeAllPlugins();

} // namespace plugin
} // namespace navsim

