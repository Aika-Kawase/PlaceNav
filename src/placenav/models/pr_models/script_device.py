"""Relocate explicit device allocations in fixed-shape traced modules."""
import io
import torch


def set_graph_device(module, device):
    def visit(block):
        for node in block.nodes():
            if node.kind() == 'prim::Constant' and str(node.output().type()) == 'Device':
                node.s_('value', str(device))
            for child in node.blocks():
                visit(child)
    for child in module.modules():
        if isinstance(child, torch.jit.ScriptModule):
            for name in child._c._method_names():
                visit(child._c._get_method(name).graph)
    # Clear cached execution plans that may still contain the old allocations.
    buffer = io.BytesIO()
    torch.jit.save(module, buffer)
    buffer.seek(0)
    module.cpu()  # Avoid keeping two full GPU copies during reload (ViT-g).
    return torch.jit.load(buffer, map_location=device)
