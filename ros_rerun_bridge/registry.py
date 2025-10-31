from __future__ import annotations

from typing import Callable, Dict, Type


class ModuleRegistry:
    """Registry for TopicToComponentModule classes."""

    def __init__(self) -> None:
        """Initialize the registry."""
        self._by_name: Dict[str, Type] = {}

    def register(self, name: str) -> Callable[[Type], Type]:
        """Decorator to register a module class under a given name."""

        def deco(cls: Type) -> Type:
            if name in self._by_name:
                raise ValueError(f"Module '{name}' already registered")
            self._by_name[name] = cls
            return cls

        return deco

    def get(self, name: str):
        """Get a registered module class by name."""
        return self._by_name[name]

    def has(self, name: str) -> bool:
        """Check if a module class is registered under the given name."""
        return name in self._by_name

    def names(self):
        """Get a sorted list of all registered module names."""
        return sorted(self._by_name)


REGISTRY = ModuleRegistry()
