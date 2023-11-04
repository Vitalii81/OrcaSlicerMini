// Include GLGizmoBase.hpp before I18N.hpp as it includes some libigl code, which overrides our localization "L" macro.
#include "GLGizmoScale.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI_App.hpp"

#include <GL/glew.h>

#include <wx/utils.h> 

namespace Slic3r {
namespace GUI {


const float GLGizmoScale3D::Offset = 5.0f;

// get intersection of ray and plane
Vec3d GetIntersectionOfRayAndPlane(Vec3d ray_position, Vec3d ray_dir, Vec3d plane_position, Vec3d plane_normal)
{
    double t = (plane_normal.dot(plane_position) - plane_normal.dot(ray_position)) / (plane_normal.dot(ray_dir));
    Vec3d  intersection = ray_position + t * ray_dir;
    return intersection;
}

//BBS: GUI refactor: add obj manipulation
GLGizmoScale3D::GLGizmoScale3D(GLCanvas3D& parent, const std::string& icon_filename, unsigned int sprite_id, GizmoObjectManipulation* obj_manipulation)
    : GLGizmoBase(parent, icon_filename, sprite_id)
    //BBS: GUI refactor: add obj manipulation
    , m_object_manipulation(obj_manipulation)
{
    m_grabber_connections[0].grabber_indices = { 0, 1 };
    m_grabber_connections[1].grabber_indices = { 2, 3 };
    m_grabber_connections[2].grabber_indices = { 4, 5 };
    m_grabber_connections[3].grabber_indices = { 6, 7 };
    m_grabber_connections[4].grabber_indices = { 7, 8 }; 
    m_grabber_connections[5].grabber_indices = { 8, 9 };
    m_grabber_connections[6].grabber_indices = { 9, 6 };
}

std::string GLGizmoScale3D::get_tooltip() const
{
    const Selection& selection = m_parent.get_selection();

    bool single_instance = selection.is_single_full_instance();
    bool single_volume = selection.is_single_modifier() || selection.is_single_volume();

    Vec3f scale = 100.0f * Vec3f::Ones();
    if (single_instance)
        scale = 100.0f * selection.get_volume(*selection.get_volume_idxs().begin())->get_instance_scaling_factor().cast<float>();
    else if (single_volume)
        scale = 100.0f * selection.get_volume(*selection.get_volume_idxs().begin())->get_volume_scaling_factor().cast<float>();

    if (m_hover_id == 0 || m_hover_id == 1 || m_grabbers[0].dragging || m_grabbers[1].dragging)
        return "X: " + format(scale.x(), 4) + "%";
    else if (m_hover_id == 2 || m_hover_id == 3 || m_grabbers[2].dragging || m_grabbers[3].dragging)
        return "Y: " + format(scale.y(), 4) + "%";
    else if (m_hover_id == 4 || m_hover_id == 5 || m_grabbers[4].dragging || m_grabbers[5].dragging)
        return "Z: " + format(scale.z(), 4) + "%";
    else if (m_hover_id == 6 || m_hover_id == 7 || m_hover_id == 8 || m_hover_id == 9 || 
        m_grabbers[6].dragging || m_grabbers[7].dragging || m_grabbers[8].dragging || m_grabbers[9].dragging)
    {
        std::string tooltip = "X: " + format(scale.x(), 2) + "%\n";
        tooltip += "Y: " + format(scale.y(), 2) + "%\n";
        tooltip += "Z: " + format(scale.z(), 2) + "%";
        return tooltip;
    }
    else
        return "";
}

void GLGizmoScale3D::enable_ununiversal_scale(bool enable)
{
    for (unsigned int i = 0; i < 6; ++i)
        m_grabbers[i].enabled = enable;
}

bool GLGizmoScale3D::on_init()
{
    for (int i = 0; i < 10; ++i) {
        m_grabbers.push_back(Grabber());
    }

    double half_pi = 0.5 * (double)PI;

    // x axis
    m_grabbers[0].angles(1) = half_pi;
    m_grabbers[1].angles(1) = half_pi;

    // y axis
    m_grabbers[2].angles(0) = half_pi;
    m_grabbers[3].angles(0) = half_pi;

    // BBS
    m_grabbers[4].enabled = false;
    m_shortcut_key = WXK_CONTROL_S;

    return true;
}

std::string GLGizmoScale3D::on_get_name() const
{
    return _u8L("Scale");
}

bool GLGizmoScale3D::on_is_activable() const
{
    const Selection& selection = m_parent.get_selection();
    return !selection.is_empty() && !selection.is_wipe_tower();
}

void GLGizmoScale3D::on_start_dragging()
{
    if (m_hover_id != -1) {
        m_starting.drag_position = m_grabbers[m_hover_id].center;
        m_starting.plane_center = m_grabbers[4].center;
        m_starting.plane_nromal = m_grabbers[5].center - m_grabbers[4].center;
        m_starting.ctrl_down = wxGetKeyState(WXK_CONTROL);
        m_starting.box = (m_starting.ctrl_down && (m_hover_id < 6)) ? m_box : m_parent.get_selection().get_bounding_box();

        const Vec3d& center = m_starting.box.center();
        m_starting.pivots[0] = m_transform * Vec3d(m_starting.box.max.x(), center.y(), center.z());
        m_starting.pivots[1] = m_transform * Vec3d(m_starting.box.min.x(), center.y(), center.z());
        m_starting.pivots[2] = m_transform * Vec3d(center.x(), m_starting.box.max.y(), center.z());
        m_starting.pivots[3] = m_transform * Vec3d(center.x(), m_starting.box.min.y(), center.z());
        m_starting.pivots[4] = m_transform * Vec3d(center.x(), center.y(), m_starting.box.max.z());
        m_starting.pivots[5] = m_transform * Vec3d(center.x(), center.y(), m_starting.box.min.z());
    }
}

void GLGizmoScale3D::on_update(const UpdateData& data)
{
    if (m_hover_id == 0 || m_hover_id == 1)
        do_scale_along_axis(X, data);
    else if (m_hover_id == 2 || m_hover_id == 3)
        do_scale_along_axis(Y, data);
    else if (m_hover_id == 4 || m_hover_id == 5)
        do_scale_along_axis(Z, data);
    else if (m_hover_id >= 6)
        do_scale_uniform(data);
}

void GLGizmoScale3D::on_render()
{
    const Selection& selection = m_parent.get_selection();

    bool single_instance = selection.is_single_full_instance();
    bool single_volume = selection.is_single_modifier() || selection.is_single_volume();

    glsafe(::glClear(GL_DEPTH_BUFFER_BIT));
    glsafe(::glEnable(GL_DEPTH_TEST));

    m_box.reset();
    m_transform = Transform3d::Identity();
    // Transforms grabbers' offsets to world refefence system 
    Transform3d offsets_transform = Transform3d::Identity();
    m_offsets_transform = Transform3d::Identity();
    Vec3d angles = Vec3d::Zero();

    if (single_instance) {
        // calculate bounding box in instance local reference system
        const Selection::IndicesList& idxs = selection.get_volume_idxs();
        for (unsigned int idx : idxs) {
            const GLVolume* vol = selection.get_volume(idx);
            m_box.merge(vol->bounding_box().transformed(vol->get_volume_transformation().get_matrix()));
        }

        // gets transform from first selected volume
        const GLVolume* v = selection.get_volume(*idxs.begin());
        m_transform = v->get_instance_transformation().get_matrix();
        // gets angles from first selected volume
        angles = v->get_instance_rotation();
        // consider rotation+mirror only components of the transform for offsets
        offsets_transform = Geometry::assemble_transform(Vec3d::Zero(), angles, Vec3d::Ones(), v->get_instance_mirror());
        m_offsets_transform = offsets_transform;
    }
    else if (single_volume) {
        const GLVolume* v = selection.get_volume(*selection.get_volume_idxs().begin());
        m_box = v->bounding_box();
        m_transform = v->world_matrix();
        angles = Geometry::extract_euler_angles(m_transform);
        // consider rotation+mirror only components of the transform for offsets
        offsets_transform = Geometry::assemble_transform(Vec3d::Zero(), angles, Vec3d::Ones(), v->get_instance_mirror());
        m_offsets_transform = Geometry::assemble_transform(Vec3d::Zero(), v->get_volume_rotation(), Vec3d::Ones(), v->get_volume_mirror());
    }
    else
        m_box = selection.get_bounding_box();

    const Vec3d& center = m_box.center();
    Vec3d offset_x = offsets_transform * Vec3d((double)Offset, 0.0, 0.0);
    Vec3d offset_y = offsets_transform * Vec3d(0.0, (double)Offset, 0.0);
    Vec3d offset_z = offsets_transform * Vec3d(0.0, 0.0, (double)Offset);

    bool ctrl_down = (m_dragging && m_starting.ctrl_down) || (!m_dragging && wxGetKeyState(WXK_CONTROL));

    // x axis
    m_grabbers[0].center = m_transform * Vec3d(m_box.min.x(), center.y(), m_box.min.z());
    m_grabbers[1].center = m_transform * Vec3d(m_box.max.x(), center.y(), m_box.min.z());

    // y axis
    m_grabbers[2].center = m_transform * Vec3d(center.x(), m_box.min.y(), m_box.min.z());
    m_grabbers[3].center = m_transform * Vec3d(center.x(), m_box.max.y(), m_box.min.z());

    // z axis do not show 4
    m_grabbers[4].center = m_transform * Vec3d(center.x(), center.y(), m_box.min.z());
    m_grabbers[4].enabled = false;

    m_grabbers[5].center = m_transform * Vec3d(center.x(), center.y(), m_box.max.z());

    // uniform
    m_grabbers[6].center = m_transform * Vec3d(m_box.min.x(), m_box.min.y(), m_box.min.z());
    m_grabbers[7].center = m_transform * Vec3d(m_box.max.x(), m_box.min.y(), m_box.min.z());
    m_grabbers[8].center = m_transform * Vec3d(m_box.max.x(), m_box.max.y(), m_box.min.z());
    m_grabbers[9].center = m_transform * Vec3d(m_box.min.x(), m_box.max.y(), m_box.min.z());

    for (int i = 0; i < 6; ++i) {
        m_grabbers[i].color       = AXES_COLOR[i/2];
        m_grabbers[i].hover_color = AXES_HOVER_COLOR[i/2];
    }

    for (int i = 6; i < 10; ++i) {
        m_grabbers[i].color       = GRABBER_UNIFORM_COL;
        m_grabbers[i].hover_color = GRABBER_UNIFORM_HOVER_COL;
    }

    // sets grabbers orientation
    for (int i = 0; i < 10; ++i) {
        m_grabbers[i].angles = angles;
    }

    glsafe(::glLineWidth((m_hover_id != -1) ? 2.0f : 1.5f));

    const BoundingBoxf3& selection_box = selection.get_bounding_box();

    float grabber_mean_size = (float)((selection_box.size().x() + selection_box.size().y() + selection_box.size().z()) / 3.0);

    //draw connections
    GLShaderProgram* shader = wxGetApp().get_shader("flat");
    if (shader != nullptr) {
        shader->start_using();
        // BBS: when select multiple objects, uniform scale can be deselected, display the connection(4,5)
        //if (single_instance || single_volume) {
        if (m_grabbers[4].enabled && m_grabbers[5].enabled)
            render_grabbers_connection(4, 5, m_grabbers[4].color);
        render_grabbers_connection(6, 7, m_grabbers[2].color);
        render_grabbers_connection(7, 8, m_grabbers[2].color);
        render_grabbers_connection(8, 9, m_grabbers[0].color);
        render_grabbers_connection(9, 6, m_grabbers[0].color);
        shader->stop_using();
    }

    // draw grabbers
    render_grabbers(grabber_mean_size);
}

void GLGizmoScale3D::on_render_for_picking()
{
    glsafe(::glDisable(GL_DEPTH_TEST));
    render_grabbers_for_picking(m_parent.get_selection().get_bounding_box());
}

void GLGizmoScale3D::render_grabbers_connection(unsigned int id_1, unsigned int id_2, const ColorRGBA& color)
{
    auto grabber_connection = [this](unsigned int id_1, unsigned int id_2) {
        for (int i = 0; i < int(m_grabber_connections.size()); ++i) {
            if (m_grabber_connections[i].grabber_indices.first == id_1 && m_grabber_connections[i].grabber_indices.second == id_2)
                return i;
        }
        return -1;
    };

    int id = grabber_connection(id_1, id_2);
    if (id == -1)
        return;

    if (!m_grabber_connections[id].model.is_initialized() ||
        !m_grabber_connections[id].old_v1.isApprox(m_grabbers[id_1].center) ||
        !m_grabber_connections[id].old_v2.isApprox(m_grabbers[id_2].center)) {
        m_grabber_connections[id].old_v1 = m_grabbers[id_1].center;
        m_grabber_connections[id].old_v2 = m_grabbers[id_2].center;
        m_grabber_connections[id].model.reset();

        GLModel::InitializationData init_data;
        GUI::GLModel::InitializationData::Entity entity;
        entity.type = GUI::GLModel::PrimitiveType::Lines;
        entity.positions.reserve(2);
        entity.positions.emplace_back(m_grabbers[id_1].center.cast<float>());
        entity.positions.emplace_back(m_grabbers[id_2].center.cast<float>());

        entity.normals.reserve(2);
        for (size_t j = 0; j < 2; ++j) {
            entity.normals.emplace_back(Vec3f::UnitZ());
        }

        entity.indices.reserve(2);
        entity.indices.emplace_back(0);
        entity.indices.emplace_back(1);

        init_data.entities.emplace_back(entity);
        m_grabber_connections[id].model.init_from(init_data);
    }

    m_grabber_connections[id].model.set_color(-1, color);
    glLineStipple(1, 0x0FFF);
    glEnable(GL_LINE_STIPPLE);
    m_grabber_connections[id].model.render();
    glDisable(GL_LINE_STIPPLE);
}

//BBS: add input window for move
void GLGizmoScale3D::on_render_input_window(float x, float y, float bottom_limit)
{
    if (m_object_manipulation)
        m_object_manipulation->do_render_scale_input_window(m_imgui, "Scale", x, y, bottom_limit);
}

void GLGizmoScale3D::do_scale_along_axis(Axis axis, const UpdateData& data)
{
    double ratio = calc_ratio(data);
    if (ratio > 0.0) {
        m_scale(axis) = m_starting.scale(axis) * ratio;
        if (m_starting.ctrl_down) {
            double local_offset = 0.5 * (m_scale(axis) - m_starting.scale(axis)) * m_starting.box.size()(axis);
            if (m_hover_id == 2 * axis)
                local_offset *= -1.0;

            Vec3d local_offset_vec;
            switch (axis)
            {
            case X: { local_offset_vec = local_offset * Vec3d::UnitX(); break; }
            case Y: { local_offset_vec = local_offset * Vec3d::UnitY(); break; }
            case Z: { local_offset_vec = local_offset * Vec3d::UnitZ(); break; }
            default: break;
            }

            m_offset = m_offsets_transform * local_offset_vec;
        }
        else
            m_offset = Vec3d::Zero();
    }
}

void GLGizmoScale3D::do_scale_uniform(const UpdateData& data)
{
    double ratio = calc_ratio(data);
    if (ratio > 0.0) {
        m_scale = m_starting.scale * ratio;
        m_offset = Vec3d::Zero();
    }
}

double GLGizmoScale3D::calc_ratio(const UpdateData& data) const
{
    double ratio = 0.0;

    Vec3d pivot = (m_starting.ctrl_down && m_hover_id < 6) ? m_starting.pivots[m_hover_id] : m_starting.plane_center;

    Vec3d starting_vec = m_starting.drag_position - pivot;
    double len_starting_vec = starting_vec.norm();
    if (len_starting_vec != 0.0) {
        Vec3d mouse_dir = data.mouse_ray.unit_vector();
        Vec3d plane_normal = m_starting.plane_nromal;
        if (m_hover_id == 5) {
            // get z-axis moving plane normal
            Vec3d plane_vec = mouse_dir.cross(m_starting.plane_nromal);
            plane_normal    = plane_vec.cross(m_starting.plane_nromal);
        }

        // finds the intersection of the mouse ray with the plane that the drag point moves
        // use ray-plane intersection see i.e. https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
        Vec3d inters = GetIntersectionOfRayAndPlane(data.mouse_ray.a, mouse_dir, m_starting.drag_position, plane_normal.normalized());

        Vec3d inters_vec = inters - m_starting.drag_position;

        // finds projection of the vector along the staring direction
        double proj = inters_vec.dot(starting_vec.normalized());

        ratio = (len_starting_vec + proj) / len_starting_vec;
    }

    if (wxGetKeyState(WXK_SHIFT))
        ratio = m_snap_step * (double)std::round(ratio / m_snap_step);

    return ratio;
}

} // namespace GUI
} // namespace Slic3r
