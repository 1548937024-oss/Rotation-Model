"""显示层一阶低通滤波。

只用于上位机显示与百分比换算，不改变下行控制量，也不替代固件保护。
默认对只读 Max Iq / Max Speed 回读值、实测转速/Iq/母线电压/温度/相电流做平滑，
避免总线偶发丢帧或量化跳变导致的显示抖动。
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class ExponentialFilter:
    """一阶低通（指数移动平均）。

    `alpha` 越大跟随越快、平滑越弱；`alpha=1` 等价于不滤波。
    取 0.2~0.4 通常能在 20 ms 轮询下得到平稳读数。
    """

    alpha: float = 0.3
    _value: float | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        self.set_alpha(self.alpha)

    def set_alpha(self, alpha: float) -> None:
        self.alpha = min(1.0, max(0.05, float(alpha)))

    @property
    def value(self) -> float | None:
        return self._value

    def reset(self) -> None:
        self._value = None

    def update(self, sample: float) -> float:
        sample = float(sample)
        if self._value is None:
            self._value = sample
        else:
            self._value += self.alpha * (sample - self._value)
        return self._value

    def update_int(self, sample: int) -> int:
        """更新并返回四舍五入后的整数，适合 mA / rpm / mV / count。"""

        return int(round(self.update(sample)))


class DisplayFilters:
    """界面使用的滤波组，统一控制开关与平滑系数。"""

    def __init__(self, alpha: float = 0.3) -> None:
        self.enabled = False
        self._alpha = alpha
        self._filters: dict[str, ExponentialFilter] = {}

    @property
    def alpha(self) -> float:
        return self._alpha

    def set_alpha(self, alpha: float) -> None:
        self._alpha = min(1.0, max(0.05, float(alpha)))
        for item in self._filters.values():
            item.set_alpha(self._alpha)

    def reset(self, *keys: str) -> None:
        if not keys:
            for item in self._filters.values():
                item.reset()
            return
        for key in keys:
            if key in self._filters:
                self._filters[key].reset()

    def apply_int(self, key: str, sample: int) -> int:
        """按 key 维护一路滤波器；关闭滤波时直接返回原始采样。"""

        if not self.enabled:
            item = self._filters.get(key)
            if item is not None:
                item.reset()
            return int(sample)
        item = self._filters.get(key)
        if item is None:
            item = ExponentialFilter(self._alpha)
            self._filters[key] = item
        return item.update_int(int(sample))

    def apply_float(self, key: str, sample: float) -> float:
        if not self.enabled:
            item = self._filters.get(key)
            if item is not None:
                item.reset()
            return float(sample)
        item = self._filters.get(key)
        if item is None:
            item = ExponentialFilter(self._alpha)
            self._filters[key] = item
        return item.update(float(sample))
