// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

// Copyright (c) 2026 FineNav Team
// Based on the architectural concepts of Nav2 SimpleActionServer
// Specialized for lightweight SDK integration.

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <functional>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::util {

/**
 * @brief FineNav 专用简化版 Action Server
 * * 核心职责：
 * 1. 维护 "Current" (当前) 和 "Pending" (挂起) 两个目标槽位。
 * 2. 处理多线程环境下的目标抢占 (Preemption) 逻辑。
 * 3. 将复杂的 Action 回调简化为单一的 execute_callback 循环。
 */
template<typename ActionT>
class SimpleActionServer
{
  public:
    // 定义回调函数类型，用户只需提供一个无参数的执行函数
    using ExecuteCallback = std::function<void()>;
    using GoalHandle = typename rclcpp_action::ServerGoalHandle<ActionT>;
    // using GoalHandlePtr = typename GoalHandle::SharedPtr;

    /**
     * @brief 构造函数
     * @param node ROS 节点句柄 (直接使用 rclcpp::Node，去除 nav2_util 依赖)
     * @param action_name Action 名字
     * @param execute_callback 用户的主执行逻辑
     */
    SimpleActionServer(rclcpp::Node::SharedPtr node, const std::string & action_name, ExecuteCallback execute_callback)
        : node_(node), action_name_(action_name), execute_callback_(execute_callback)
    {
        using namespace std::placeholders;
        action_server_ = rclcpp_action::create_server<ActionT>(
            node_,
            action_name_,
            std::bind(&SimpleActionServer::handle_goal, this, _1, _2),
            std::bind(&SimpleActionServer::handle_cancel, this, _1),
            std::bind(&SimpleActionServer::handle_accepted, this, _1)
        );

        FINENAV_LOG_INFO(node_->get_logger(), "Action server started.");
    }

    ~SimpleActionServer()
    {
        // 逻辑：析构时必须确保工作线程退出，否则会抛出 std::terminate
        {
            std::lock_guard<std::recursive_mutex> lock(update_mutex_);
            stop_execution_ = true;
            // abort() first moves the goal to a terminal state, then publishes
            // the result.  The publish can fail after shutdown ("Asked to
            // publish result for goal that does not exist"), but _abort() has
            // already run and that is the part we need: a goal left active is
            // published from ~ServerGoalHandle, whose deleter runs inside the
            // noexcept _M_dispose() of the shared_ptr and cannot be caught --
            // it would call std::terminate.  So always abort, even when the
            // context is already gone, and swallow only the publish failure.
            if (current_handle_ && current_handle_->is_active()) {
                try {
                    current_handle_->abort(std::make_shared<typename ActionT::Result>());
                } catch (const std::exception & ex) {
                    FINENAV_LOG_INFO(node_->get_logger(), "abort() during shutdown: {}", ex.what());
                }
            }
            // Releasing a GoalHandle then runs rclcpp_action's deleter only for
            // a still-active goal, which cannot happen above.  Drop the handles
            // here anyway so their release is not left to the member destructors.
            try {
                current_handle_.reset();
            } catch (const std::exception &) {
            }
            try {
                pending_handle_.reset();
            } catch (const std::exception &) {
            }
            // The action server keeps its own reference to every goal handle,
            // so its destruction releases the last one and runs the same
            // deleter.  Destroy it here, inside the same guard.
            try {
                action_server_.reset();
            } catch (const std::exception &) {
            }
        }
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    /**
     * @brief 检查是否有新目标正在排队（抢占请求）
     * 逻辑：用户在 execute 循环中必须频繁检查此标志。
     * 如果返回 true，用户应尽快退出当前循环，以便 worker 线程调度新目标。
     */
    bool is_preempt_requested() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        return pending_handle_ != nullptr;
    }

    /**
     * @brief 检查是否收到取消请求
     */
    bool is_cancel_requested() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        // 逻辑：无论是当前目标被取消，还是排队目标被取消，都视为取消信号
        if (current_handle_ && current_handle_->is_canceling()) return true;
        if (pending_handle_ && pending_handle_->is_canceling()) return true;
        return false;
    }

    /**
     * @brief 获取当前正在执行的目标
     */
    std::shared_ptr<const typename ActionT::Goal> get_current_goal() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (!current_handle_ || !current_handle_->is_active()) {
            return nullptr;
        }
        return current_handle_->get_goal();
    }

    /**
     * @brief 接受挂起的目标 (执行抢占切换)
     * 逻辑：
     * 1. 终止当前目标 (Canceled/Aborted)
     * 2. 将 Pending 提升为 Current
     * 3. 返回新目标的 Handle 供用户获取数据
     */
     const std::shared_ptr<const typename ActionT::Goal>  accept_pending_goal()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!pending_handle_) {
            FINENAV_LOG_ERROR(node_->get_logger(), "No pending goal to accept.");
            return nullptr;
        }

        // 如果当前还有旧目标在跑，先把它结束掉
        if (current_handle_ && current_handle_->is_active() && current_handle_ != pending_handle_) {
            FINENAV_LOG_INFO(node_->get_logger(), "Preempting current goal.");
            auto result = std::make_shared<typename ActionT::Result>();
            current_handle_->abort(result); // 或者 canceled，视业务逻辑而定，通常抢占视为 abort 旧的
        }

        current_handle_ = pending_handle_;
        pending_handle_.reset();

        return current_handle_->get_goal();
    }

    /**
     * @brief 标记当前任务成功
     */
    void succeeded_current(typename std::shared_ptr<typename ActionT::Result> result =
                           std::make_shared<typename ActionT::Result>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (current_handle_ && current_handle_->is_active()) {
            current_handle_->succeed(result);
        }
    }

    /**
     * @brief 终止所有任务 (用于 Cancel 或 Shutdown)
     */
    void terminate_all(typename std::shared_ptr<typename ActionT::Result> result =
                       std::make_shared<typename ActionT::Result>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (current_handle_ && current_handle_->is_active()) {
            current_handle_->abort(result);
        }
        if (pending_handle_ && pending_handle_->is_active()) {
            pending_handle_->abort(result);
        }
        pending_handle_.reset();
    }

    /**
     * @brief 发布反馈
     */
    void publish_feedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (current_handle_ && current_handle_->is_active()) {
            current_handle_->publish_feedback(feedback);
        }
    }

  protected:
    // ========================================================================
    // 内部 ROS Action 回调 (Internal Callbacks)
    // ========================================================================

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & /*uuid*/,
        std::shared_ptr<const typename ActionT::Goal> /*goal*/)
    {
        // 逻辑：总是接受新目标。FineNav 的策略是"喜新厌旧"，不拒绝请求。
        // 可以在这里加一个 is_server_active 检查，如果需要支持停机维护的话。
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> handle)
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        // 逻辑：总是允许取消。
        if (!handle->is_active()) {
            return rclcpp_action::CancelResponse::REJECT;
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * @brief 核心状态机入口：处理已被 ROS 接受的目标
     */
    void handle_accepted(const std::shared_ptr<GoalHandle> handle)
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        // If current goal is active, put new goal into pending slot
        if (current_handle_ && current_handle_->is_active()) {
            FINENAV_LOG_INFO(node_->get_logger(), "New goal received while busy, storing as pending.");

            // If there's already a pending goal, abort it
            if (pending_handle_ && pending_handle_->is_active()) {
                FINENAV_LOG_WARN(node_->get_logger(), "Overwriting previous pending goal.");
                auto result = std::make_shared<typename ActionT::Result>();
                pending_handle_->abort(result);
            }
            pending_handle_ = handle;
        }
        // Else, start executing the new goal immediately
        else {
            FINENAV_LOG_INFO(node_->get_logger(), "New goal received, starting execution.");
            current_handle_ = handle;

            if (worker_thread_.joinable()) { // Expeted that thread is not running
                worker_thread_.join();
            }
            stop_execution_ = false; // TODO: 将 worker_thread_ 改为常驻线程，结合条件变量管理
            worker_thread_ = std::thread(std::bind(&SimpleActionServer::work, this));
        }
    }

    /**
     * @brief 工作线程主循环 (Worker Loop)
     * 逻辑：这个循环保证了任务的连续执行。只要还有 Pending 任务，线程就不会退出。
     */
    void work()
    {
        while (rclcpp::ok() && !stop_execution_) {
            // Execute the current goal's callback
            try {
                if (current_handle_ && current_handle_->is_active()) {
                    execute_callback_();
                }
            } catch (const std::exception & ex) {
                FINENAV_LOG_ERROR(node_->get_logger(), "Exception in execution callback: {}", ex.what());
                terminate_all();
                return;
            }

            std::lock_guard<std::recursive_mutex> lock(update_mutex_);

            // If current goal is still active, but not set to succeeded, abort it.
            // Always attempt this: abort() terminalizes the goal before it
            // publishes, and only the terminal state protects the later handle
            // destructor from publishing inside a noexcept deleter.
            if (current_handle_ && current_handle_->is_active()) {
                if (rclcpp::ok()) {
                    FINENAV_LOG_WARN(node_->get_logger(), "Goal finished but result not set, aborting.");
                } else {
                    FINENAV_LOG_INFO(node_->get_logger(), "Shutdown requested; aborting the active goal.");
                }
                try {
                    current_handle_->abort(std::make_shared<typename ActionT::Result>());
                } catch (const std::exception & ex) {
                    FINENAV_LOG_WARN(node_->get_logger(), "abort() failed: {}", ex.what());
                }
            }

            // If there's a pending goal, promote it to current, and continue loop
            if (pending_handle_) {
                FINENAV_LOG_INFO(node_->get_logger(), "Processing pending goal immediately.");
                current_handle_ = pending_handle_;
                pending_handle_.reset();
            } else {
                break;
            }
        }
    }

  private:
    rclcpp::Node::SharedPtr node_;
    std::string action_name_;
    ExecuteCallback execute_callback_;

    typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;

    mutable std::recursive_mutex update_mutex_;

    std::shared_ptr<GoalHandle> current_handle_; // need mutex
    std::shared_ptr<GoalHandle> pending_handle_; // need mutex

    std::thread worker_thread_;
    std::atomic<bool> stop_execution_{false};
};

} // namespace finenav
