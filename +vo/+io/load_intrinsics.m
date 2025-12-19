function K = load_intrinsics(path)
S = load(path);
fn = fieldnames(S);

K = [];
for i = 1:numel(fn)
    if isa(S.(fn{i}), "cameraIntrinsics")
        K = S.(fn{i});
        return;
    end
end

error("intrinsics.mat 안에 cameraIntrinsics 객체가 없습니다.");
end
