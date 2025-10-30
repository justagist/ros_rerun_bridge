from __future__ import annotations

from typing import Callable, Dict, Type


class ModuleRegistry:
    def __init__(self) -> None:
        self._by_name: Dict[str, Type] = {}

    def register(self, name: str) -> Callable[[Type], Type]:
        def deco(cls: Type) -> Type:
            if name in self._by_name:
                raise ValueError(f"Module '{name}' already registered")
            self._by_name[name] = cls
            return cls
        return deco

    def get(self, name: str):
        return self._by_name[name]

    def has(self, name: str) -> bool:
        return name in self._by_name

    def names(self):
        return sorted(self._by_name)


REGISTRY = ModuleRegistry()
